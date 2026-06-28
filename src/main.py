"""
main.py -- mmWave arm safety pipeline
DNTD Dynamics -- IWR6843AOP

Wires together:
  uart_reader.py      -> MmwaveReader (background thread, queue of parsed Frame objects)
  tlv_parser.py       -> Frame/Point dataclasses (already parsed inside MmwaveReader)
  zone_logic.py       -> CLEAR / CAUTION / STOP classification
  background_model.py -> voxel-grid background learner (optional, --bg-learn)
  presence_hold.py    -> STOP-triggered static presence monitor (always active)
  ZoneOutputs         -> serial + GPIO + MQTT

Filtering pipeline (applied per point before classification):
  1. min-range    -- drops near-field mount/self-reflection clutter
  2. min-velocity -- drops near-static returns (walls, furniture).
                     NOTE: also filters a perfectly still person. The
                     StaticPresenceHold layer above corrects for this --
                     it operates on the unfiltered point list so it can
                     see the low-velocity sway returns the filter drops.

Background learning (--bg-learn):
  On first run, spends --bg-learn-time seconds learning the static environment.
  After learning, only novel objects (people, tools) trigger zone changes. A
  person who walks in and stands still remains detected -- they are never
  absorbed into the background map. The learned map is saved and reloaded on
  subsequent runs, skipping the learning phase entirely.
  Use --bg-relearn to force a fresh cycle after moving the sensor.

Static presence hold (always active):
  When STOP is confirmed, the pipeline latches and holds STOP even after
  the person stops moving. Two evidence sources keep the hold active:
    1. Background model novelty -- novel voxel in hazard zone (if --bg-learn)
    2. Micro-Doppler sway -- low-amplitude returns (~0.02-0.25 m/s) from
       a person's involuntary body movement while standing still
  The hold releases only after both signals are absent for --hold-timeout
  seconds, followed by a brief release grace period.

Occupancy hold (--occupancy-hold, always active):
  Bridges the gap between when points drop below the velocity filter and
  when StaticPresenceHold has enough sway history to take over. Holds the
  last-seen zone for a short time-window on empty frames.

Usage:
  python3 main.py --dry-run
  python3 main.py --serial /dev/ttyACM0
  python3 main.py --mqtt 192.168.254.117
  python3 main.py --bg-learn --dry-run
  python3 main.py --bg-relearn --dry-run
  python3 main.py --help
"""

import argparse
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import replace

# ---------------------------------------------------------------------------
# Logging -- configure before imports so all modules use the same format
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")

# ---------------------------------------------------------------------------
# Local imports
# ---------------------------------------------------------------------------
try:
    from uart_reader import MmwaveReader
    from zone_logic import ZoneClassifier, ZoneOutputs, ZoneState, DetectedPoint
    from presence_hold import StaticPresenceHold
except ImportError as e:
    logger.error(f"Import failed: {e}")
    logger.error(
        "Make sure uart_reader.py, zone_logic.py, presence_hold.py "
        "are in the same directory."
    )
    sys.exit(1)

# Background model imported only when --bg-learn is set.
# Keeps the base install numpy-free.


# ---------------------------------------------------------------------------
# Hardware config -- sensor ports (match your Jetson enumeration)
# ---------------------------------------------------------------------------
CLI_PORT    = "/dev/ttyUSB0"
CLI_BAUD    = 115200
DATA_PORT   = "/dev/ttyUSB1"
DATA_BAUD   = 921600
_REPO_ROOT  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_FILE = os.path.join(_REPO_ROOT, "configs", "profile_AOP.cfg")

# Zone severity for occupancy-hold comparison
ZONE_SEVERITY = {"CLEAR": 0, "CAUTION": 1, "STOP": 2}


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------
def run(args):
    logger.info("DNTD Dynamics -- mmWave arm safety pipeline starting")
    logger.info(f"CLI:  {CLI_PORT} @ {CLI_BAUD}")
    logger.info(f"Data: {DATA_PORT} @ {DATA_BAUD}")
    logger.info(f"Min range filter:    {args.min_range}m")
    logger.info(f"Min velocity filter: {args.min_velocity}m/s")
    logger.info(f"Hysteresis: upgrade={args.hysteresis} frames, downgrade={args.clear_hysteresis} frames")
    logger.info(f"Occupancy hold: {args.occupancy_hold}s")

    # --- Optional background model ---
    bg_model = None
    if args.bg_learn:
        try:
            from background_model import BackgroundModel
        except ImportError:
            logger.error(
                "background_model.py requires numpy.  "
                "Install with: pip3 install numpy"
            )
            sys.exit(1)

        bg_model = BackgroundModel(
            learning_duration_s = args.bg_learn_time,
            map_path            = args.bg_map_path,
        )

        if args.bg_relearn:
            logger.info("--bg-relearn: clearing saved map and forcing fresh learning cycle")
            bg_model.start_relearn()

        if bg_model.state == "ACTIVE":
            logger.info(
                "Background map loaded from disk -- "
                "skipping learning phase, active immediately"
            )
        else:
            logger.info(
                f"Background learning: {args.bg_learn_time:.0f}s -- "
                "stand clear of the sensor workspace during this phase"
            )

    # --- Zone classifier ---
    classifier = ZoneClassifier(
        stop_range       = args.stop_range,
        caution_range    = args.caution_range,
        fast_approach    = args.fast_approach,
        static_filter    = args.min_velocity,   # now also enforced inside classifier (bug fix)
        hysteresis       = args.hysteresis,
        clear_hysteresis = args.clear_hysteresis,
    )

    # --- Static presence hold (always active) ---
    presence = StaticPresenceHold(
        background_model = bg_model,
        hazard_radius_m  = args.caution_range + 0.2,
        hold_timeout_s   = args.hold_timeout,
        release_grace_s  = args.release_grace,
    )

    # --- Output layer ---
    outputs = ZoneOutputs(
        serial_port = args.serial if not args.dry_run else None,
        use_gpio    = args.gpio   if not args.dry_run else False,
        mqtt_broker = args.mqtt   if not args.dry_run else None,
    )

    if args.dry_run:
        logger.info("DRY RUN -- zone states will print only, no outputs active")

    # --- mmWave reader ---
    reader = MmwaveReader(
        cli_port  = CLI_PORT,
        cli_baud  = CLI_BAUD,
        data_port = DATA_PORT,
        data_baud = DATA_BAUD,
    )

    logger.info(f"Sending sensor config: {CONFIG_FILE}")
    errors = reader.send_config(CONFIG_FILE)
    if errors:
        for line, resp in errors:
            logger.error(f"Config error on '{line}': {resp}")
        logger.error("Sensor config failed -- aborting")
        sys.exit(1)

    logger.info("Config sent. Starting reader thread...")
    reader.start()

    # --- Graceful shutdown ---
    stop_event = threading.Event()

    def _shutdown(sig, frame):
        logger.info("Shutdown signal received -- stopping pipeline")
        stop_event.set()

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # --- Main loop ---
    frame_count    = 0
    last_log_t     = time.time()
    last_zone      = None
    min_range_sq   = args.min_range ** 2
    min_velocity   = args.min_velocity

    # Occupancy hold state
    last_seen_zone = None
    last_seen_time = 0.0

    # Background learning flag -- used to log the transition exactly once
    was_learning = (bg_model is not None and bg_model.state == "LEARNING")

    try:
        while not stop_event.is_set():
            frame = reader.get_frame(timeout=1.0)
            if frame is None:
                continue

            # frame.points are tlv_parser.Point objects (already parsed by MmwaveReader).
            # Keep a full unfiltered copy for background model and presence hold,
            # then build the velocity+range filtered list for the zone classifier.
            all_points = [
                DetectedPoint(x=p.x, y=p.y, z=p.z, velocity=p.velocity, snr=p.snr)
                for p in frame.points
            ]

            # Background model observes all points (no velocity filter --
            # it needs to see static returns to learn what's background)
            if bg_model is not None:
                bg_model.observe(all_points)

                if bg_model.state == "LEARNING":
                    stats = bg_model.get_stats()
                    remaining = stats.seconds_remaining
                    # Log progress every ~5s worth of frames
                    if frame_count % 50 == 0:
                        logger.info(
                            f"🔵 Background learning: {remaining:.0f}s remaining -- "
                            "arm held at STOP"
                        )
                    # Hold STOP during learning -- can't classify yet
                    learning_state = ZoneState(
                        zone="STOP",
                        reason=f"Background learning: {remaining:.0f}s remaining",
                        closest_m=None,
                        fastest_approach_mps=None,
                        point_count=len(all_points),
                    )
                    outputs.publish(learning_state)
                    last_zone = "STOP"
                    frame_count += 1
                    continue
                elif was_learning:
                    # Just finished learning -- log the transition
                    logger.info("✅ Background learning complete -- pipeline now active")
                    was_learning = False

            # Filtered point list: range exclusion + velocity gate
            # (velocity gate now also enforced inside ZoneClassifier._classify_points)
            points = [
                p for p in all_points
                if (p.x*p.x + p.y*p.y + p.z*p.z) >= min_range_sq
                and abs(p.velocity) >= min_velocity
            ]

            # Zone classification
            state = classifier.update_frame(points)

            # --- Occupancy hold ---
            # Holds the last-seen zone for a short window after detections drop.
            # Bridges the gap before StaticPresenceHold's sway history builds up.
            now = time.time()
            if state.point_count > 0:
                last_seen_zone = state.zone
                last_seen_time = now
            elif (
                last_seen_zone is not None
                and (now - last_seen_time) < args.occupancy_hold
                and ZONE_SEVERITY[last_seen_zone] > ZONE_SEVERITY[state.zone]
            ):
                state = replace(
                    state,
                    zone=last_seen_zone,
                    reason=f"Holding {last_seen_zone} -- last detection {now - last_seen_time:.1f}s ago",
                )

            # --- Static presence hold ---
            # Operates on all_points (unfiltered) so it sees the low-velocity
            # sway returns that the velocity filter drops.
            effective_zone = presence.process(state.zone, all_points)

            # If presence hold overrode the zone, patch state for logging/output
            if effective_zone != state.zone:
                state = replace(
                    state,
                    zone=effective_zone,
                    reason=presence.hold_reason,
                )

            # Publish outputs (ZoneOutputs deduplicates on zone change)
            outputs.publish(state)

            # Console log -- always if verbose, otherwise only on zone change
            if args.verbose or state.zone != last_zone:
                marker = {"CLEAR": "✅", "CAUTION": "⚠️ ", "STOP": "🛑"}[state.zone]
                hold_tag = f" [{presence.state}]" if presence.state != "IDLE" else ""
                logger.info(
                    f"{marker}  ZONE -> {state.zone}{hold_tag}  |  {state.reason}"
                )
                last_zone = state.zone

            frame_count += 1

            # Periodic stats every 10s
            now2 = time.time()
            if now2 - last_log_t >= 10.0:
                fps = frame_count / (now2 - last_log_t) if (now2 - last_log_t) > 0 else 0
                hold_info = (
                    f" | hold={presence.state} ({presence.held_for_s:.1f}s)"
                    if presence.state != "IDLE" else ""
                )
                bg_info = ""
                if bg_model is not None:
                    s = bg_model.get_stats()
                    bg_info = f" | bg={s.state} ({s.background_voxels}vox)"
                logger.info(
                    f"Pipeline stats: {frame_count} frames | {fps:.1f} fps | "
                    f"zone={state.zone} | pts={state.point_count} | "
                    f"q={reader.frame_queue.qsize()}{hold_info}{bg_info}"
                )
                frame_count = 0
                last_log_t  = now2

    finally:
        logger.info("Stopping reader...")
        reader.stop()
        outputs.cleanup()
        logger.info("Pipeline stopped cleanly.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="mmWave arm safety pipeline -- DNTD Dynamics",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Output interfaces
    out = p.add_argument_group("Output")
    out.add_argument("--serial", metavar="PORT", default=None,
                     help="Serial port for zone output (e.g. /dev/ttyACM0)")
    out.add_argument("--gpio", action="store_true",
                     help="Output zone state on GPIO pins (Raspberry Pi BCM 17/27/22)")
    out.add_argument("--mqtt", metavar="BROKER", default=None,
                     help="MQTT broker IP for zone output (e.g. 192.168.254.117)")
    out.add_argument("--dry-run", action="store_true",
                     help="Print zone states only, activate no hardware outputs")

    # Zone geometry
    zone = p.add_argument_group("Zone geometry")
    zone.add_argument("--stop-range",    type=float, default=0.5,  metavar="M",
                      help="Hard stop radius in meters")
    zone.add_argument("--caution-range", type=float, default=1.2,  metavar="M",
                      help="Caution zone radius in meters")
    zone.add_argument("--fast-approach", type=float, default=-0.8, metavar="M/S",
                      help="Approach velocity that triggers STOP from caution zone")

    # Point filters
    filt = p.add_argument_group("Point filters")
    filt.add_argument("--min-range",    type=float, default=0.1, metavar="M",
                      help="Drop returns closer than this (mount/self-reflection exclusion)")
    filt.add_argument("--min-velocity", type=float, default=0.3, metavar="M/S",
                      help="Drop returns slower than this (static clutter rejection)")
    filt.add_argument("--hysteresis",       type=int, default=2,
                      help="Frames to confirm zone upgrade (CLEAR->CAUTION->STOP)")
    filt.add_argument("--clear-hysteresis", type=int, default=10,
                      help="Frames of no-detection before confirming CLEAR")
    filt.add_argument("--occupancy-hold", type=float, default=1.5, metavar="SEC",
                      help="Seconds to hold last-seen zone after detections stop")

    # Static presence hold
    hold = p.add_argument_group("Static presence hold")
    hold.add_argument("--hold-timeout",   type=float, default=5.0, metavar="S",
                      help="Seconds of no evidence before releasing STOP hold")
    hold.add_argument("--release-grace",  type=float, default=2.0, metavar="S",
                      help="Grace period after hold condition clears before IDLE")

    # Background learning
    bg = p.add_argument_group("Background learning (requires numpy)")
    bg.add_argument("--bg-learn",      action="store_true",
                    help="Enable background scene learning")
    bg.add_argument("--bg-learn-time", type=float, default=15.0, metavar="S",
                    help="Duration of initial background learning phase")
    bg.add_argument("--bg-relearn",    action="store_true",
                    help="Force fresh learning cycle (clear saved map and relearn)")
    bg.add_argument("--bg-map-path",
                    default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs", "background_map.npz"),
                    metavar="PATH",
                    help="Path for saved background map")

    # Diagnostics
    p.add_argument("--verbose", action="store_true",
                   help="Print zone state every frame, not just on change")

    args = p.parse_args()
    args.bg_map_path = os.path.expanduser(args.bg_map_path)
    return args


if __name__ == "__main__":
    run(parse_args())
