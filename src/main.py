"""
main.py — mmWave arm safety pipeline
DNTD Dynamics — IWR6843AOP on 6-DOF robot arm

Wires together:
  uart_reader.py   → live TLV frame queue from sensor
  tlv_parser.py    → frame decoding (x, y, z, velocity, SNR)
  zone_logic.py    → CLEAR / CAUTION / STOP classification
  ZoneOutputs      → serial + GPIO + MQTT (configure below)

Usage:
  python3 main.py                      # default config
  python3 main.py --serial /dev/ttyACM0  # output to Arduino/Pi
  python3 main.py --mqtt 192.168.254.117 # output to Sterling broker
  python3 main.py --gpio               # output on GPIO pins (RPi only)
  python3 main.py --dry-run            # print zones, no outputs

Run 'python3 main.py --help' for full options.
"""

import argparse
import logging
import queue
import signal
import sys
import time
import threading

# ---------------------------------------------------------------------------
# Configure logging before imports so all modules use the same format
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")

# ---------------------------------------------------------------------------
# Local imports — all in ~/mmwave/src/
# ---------------------------------------------------------------------------
try:
    from uart_reader import UARTReader          # your existing reader
    from tlv_parser  import parse_frame         # your existing parser
    from zone_logic  import (
        ZoneClassifier, ZoneOutputs,
        points_from_tlv_frame, DetectedPoint,
    )
except ImportError as e:
    logger.error(f"Import failed: {e}")
    logger.error("Make sure uart_reader.py, tlv_parser.py, zone_logic.py are in the same directory.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Config — sensor ports (match your Jetson enumeration)
# ---------------------------------------------------------------------------
CLI_PORT    = "/dev/ttyUSB0"
CLI_BAUD    = 115200
DATA_PORT   = "/dev/ttyUSB1"
DATA_BAUD   = 921600
CONFIG_FILE = "/home/nic/mmwave/configs/profile_AOP.cfg"


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------
def run(args):
    logger.info("DNTD Dynamics — mmWave arm safety pipeline starting")
    logger.info(f"CLI:  {CLI_PORT} @ {CLI_BAUD}")
    logger.info(f"Data: {DATA_PORT} @ {DATA_BAUD}")

    # --- Zone classifier ---
    classifier = ZoneClassifier(
        stop_range    = args.stop_range,
        caution_range = args.caution_range,
        fast_approach = args.fast_approach,
    )

    # --- Output layer ---
    outputs = ZoneOutputs(
        serial_port  = args.serial if not args.dry_run else None,
        use_gpio     = args.gpio   if not args.dry_run else False,
        mqtt_broker  = args.mqtt   if not args.dry_run else None,
    )

    if args.dry_run:
        logger.info("DRY RUN — zone states will print only, no outputs active")

    # --- Frame queue (filled by uart_reader, drained here) ---
    frame_q = queue.Queue(maxsize=20)  # drop old frames if pipeline falls behind

    # --- UART reader ---
    reader = UARTReader(
        cli_port  = CLI_PORT,
        cli_baud  = CLI_BAUD,
        data_port = DATA_PORT,
        data_baud = DATA_BAUD,
        config    = CONFIG_FILE,
        frame_queue = frame_q,
    )

    # --- Graceful shutdown ---
    stop_event = threading.Event()

    def _shutdown(sig, frame):
        logger.info("Shutdown signal received — stopping pipeline")
        stop_event.set()

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # --- Start reader thread ---
    reader_thread = threading.Thread(target=reader.run, args=(stop_event,), daemon=True)
    reader_thread.start()
    logger.info("UART reader started — waiting for first frame...")

    # --- Main loop ---
    frame_count  = 0
    last_log_t   = time.time()
    last_zone    = None

    try:
        while not stop_event.is_set():
            try:
                raw_frame = frame_q.get(timeout=1.0)
            except queue.Empty:
                continue

            # Decode TLV frame → point list
            try:
                decoded = parse_frame(raw_frame)
            except Exception as e:
                logger.debug(f"Frame decode error: {e}")
                continue

            # Convert to DetectedPoint objects
            points = points_from_tlv_frame(decoded)

            # Classify
            state = classifier.update_frame(points)

            # Publish outputs (only on zone change — ZoneOutputs handles dedup)
            outputs.publish(state)

            # Console log on zone change
            if state.zone != last_zone:
                marker = {"CLEAR": "✅", "CAUTION": "⚠️ ", "STOP": "🛑"}[state.zone]
                logger.info(f"{marker}  ZONE → {state.zone}  |  {state.reason}")
                last_zone = state.zone

            frame_count += 1

            # Periodic stats every 10s
            now = time.time()
            if now - last_log_t >= 10.0:
                fps = frame_count / (now - last_log_t) if (now - last_log_t) > 0 else 0
                logger.info(
                    f"Pipeline stats: {frame_count} frames | "
                    f"{fps:.1f} fps | zone={state.zone} | "
                    f"pts={state.point_count} | q={frame_q.qsize()}"
                )
                frame_count = 0
                last_log_t  = now

    finally:
        logger.info("Stopping reader...")
        stop_event.set()
        reader_thread.join(timeout=3.0)
        outputs.cleanup()
        logger.info("Pipeline stopped cleanly.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="mmWave arm safety pipeline — DNTD Dynamics"
    )
    p.add_argument(
        "--serial", metavar="PORT", default=None,
        help="Serial port for zone output (e.g. /dev/ttyACM0 or COM3)"
    )
    p.add_argument(
        "--gpio", action="store_true",
        help="Output zone state on GPIO pins (Raspberry Pi BCM 17/27/22)"
    )
    p.add_argument(
        "--mqtt", metavar="BROKER", default=None,
        help="MQTT broker IP for zone output (e.g. 192.168.254.117)"
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="Print zone states only, activate no outputs"
    )
    p.add_argument(
        "--stop-range", type=float, default=0.5,
        metavar="M",
        help="Hard stop radius in meters (default: 0.5)"
    )
    p.add_argument(
        "--caution-range", type=float, default=1.2,
        metavar="M",
        help="Caution zone radius in meters (default: 1.2)"
    )
    p.add_argument(
        "--fast-approach", type=float, default=-0.8,
        metavar="M/S",
        help="Approach velocity that triggers STOP from caution zone (default: -0.8)"
    )
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
