#!/usr/bin/env python3
"""
validate_hardware.py
DNTD Dynamics — RadarGuard Out-of-Box Hardware Validation

Run this first when you receive your RadarGuard kit.
Verifies the sensor is connected, streaming, and producing valid detections
before you attempt any arm mounting or pipeline configuration.

Checks performed:
  1. Port enumeration  — finds ttyUSB0/1 and confirms CP2105 device
  2. Config send       — sends profile_AOP.cfg and checks all commands ACK
  3. Frame stream      — confirms TLV frames arrive at expected 10 Hz rate
  4. Point cloud       — confirms x/y/z/velocity/SNR fields are valid
  5. Live detection    — walks you through a wave test to confirm detection
  6. SNR sanity        — warns if average SNR is below useful threshold

Usage:
  cd ~/mmwave
  python3 src/validate_hardware.py

  # If your ports are different:
  python3 src/validate_hardware.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1

Pass/fail result printed at the end. All failures include a diagnostic hint.

If this passes: your hardware is good. Proceed to main.py or the ROS2 node.
If this fails:  fix the flagged issue before going further. Don't skip steps.
"""

import argparse
import math
import sys
import time
import os
import threading

# ── Colour helpers (no dependencies) ──────────────────────────────────────
def _c(code, text): return f"\033[{code}m{text}\033[0m" if sys.stdout.isatty() else text
OK    = lambda t: _c("92", f"  ✓  {t}")
FAIL  = lambda t: _c("91", f"  ✗  {t}")
WARN  = lambda t: _c("93", f"  ⚠  {t}")
INFO  = lambda t: _c("96", f"     {t}")
HEAD  = lambda t: _c("1;96", f"\n{t}")
STEP  = lambda n, t: _c("1", f"\n[{n}] {t}")


def banner():
    print(_c("1;96", """
  ╔══════════════════════════════════════════════════════╗
  ║   RadarGuard — Out-of-Box Hardware Validation        ║
  ║   DNTD Dynamics                                      ║
  ╚══════════════════════════════════════════════════════╝
"""))


# ── Check 1: Port enumeration ──────────────────────────────────────────────

def check_ports(cli_port, data_port):
    print(STEP(1, "Port enumeration"))
    results = []

    for port, label in [(cli_port, "CLI"), (data_port, "Data")]:
        if os.path.exists(port):
            print(OK(f"{label} port found: {port}"))
            results.append(True)
        else:
            print(FAIL(f"{label} port not found: {port}"))
            print(INFO("Check: ls /dev/ttyUSB*"))
            print(INFO("Check: is the sensor plugged in and powered?"))
            print(INFO("Check: lsusb | grep 10c4  (CP2105 = Silicon Labs 10c4:ea70)"))
            results.append(False)

    # Check for CP2105 via lsusb
    try:
        import subprocess
        out = subprocess.check_output(["lsusb"], stderr=subprocess.DEVNULL).decode()
        if "10c4" in out.lower() or "silicon" in out.lower():
            print(OK("CP2105 USB bridge detected via lsusb"))
        else:
            print(WARN("CP2105 not visible in lsusb — sensor may not be powered"))
    except Exception:
        print(INFO("(lsusb check skipped — not available)"))

    return all(results)


# ── Check 2: Config send ───────────────────────────────────────────────────

def check_config(cli_port, config_path):
    print(STEP(2, f"Config send — {config_path}"))

    if not os.path.exists(config_path):
        print(FAIL(f"Config file not found: {config_path}"))
        print(INFO("Expected location: ~/mmwave/configs/profile_AOP.cfg"))
        print(INFO("Kit owners: copy profile_AOP.cfg from your private repo access"))
        return False, None

    try:
        import serial
    except ImportError:
        print(FAIL("pyserial not installed — run: pip3 install pyserial"))
        return False, None

    try:
        from uart_reader import MmwaveReader
    except ImportError:
        print(FAIL("uart_reader.py not found — run from repo root: cd ~/mmwave"))
        return False, None

    reader = MmwaveReader(cli_port=cli_port)

    print(INFO("Sending config commands..."))
    errors = reader.send_config(config_path)

    if errors:
        print(FAIL(f"{len(errors)} config command(s) returned errors:"))
        for cmd, resp in errors:
            print(INFO(f"  cmd:  {cmd}"))
            print(INFO(f"  resp: {resp}"))
        print(INFO("Common cause: wrong firmware on sensor"))
        print(INFO("Required: out_of_box_6843_aop.bin (NOT xwr68xx_mmw_demo.bin)"))
        print(INFO("Flash via TI UniFlash in SOP2 mode"))
        return False, reader
    else:
        # Count commands sent
        with open(config_path) as f:
            cmds = [l.strip() for l in f
                    if l.strip() and not l.strip().startswith('%')]
        print(OK(f"All {len(cmds)} config commands acknowledged cleanly"))
        return True, reader


# ── Check 3 + 4: Frame stream and point cloud ──────────────────────────────

def check_streaming(reader, duration=5.0):
    print(STEP(3, f"Frame stream — collecting {duration:.0f}s of data"))

    reader.start()
    time.sleep(0.5)  # let stream settle

    frames        = []
    start         = time.time()
    last_print    = start

    while time.time() - start < duration:
        frame = reader.get_frame(timeout=0.2)
        if frame is not None:
            frames.append(frame)
        now = time.time()
        if now - last_print >= 1.0:
            elapsed = now - start
            fps = len(frames) / elapsed if elapsed > 0 else 0
            print(INFO(f"  {elapsed:.0f}s — {len(frames)} frames ({fps:.1f} fps)"), end='\r')
            last_print = now

    print()
    elapsed = time.time() - start
    fps     = len(frames) / elapsed if elapsed > 0 else 0

    if len(frames) == 0:
        print(FAIL("No frames received"))
        print(INFO("Check: sensorStart was the last command in the config"))
        print(INFO("Check: data port baud rate is 921600"))
        print(INFO("Check: CLI and data ports are not swapped"))
        return False, []

    if fps < 8.0:
        print(WARN(f"Frame rate low: {fps:.1f} fps (expect ~10 Hz)"))
        print(INFO("May improve after a few seconds — check USB connection quality"))
    else:
        print(OK(f"Frame stream healthy: {fps:.1f} fps over {elapsed:.0f}s ({len(frames)} frames)"))

    return True, frames


def check_point_cloud(frames):
    print(STEP(4, "Point cloud field validation"))

    frames_with_points = [f for f in frames if f.points]
    total_points       = sum(len(f.points) for f in frames)

    if not frames_with_points:
        print(WARN("No points detected in any frame"))
        print(INFO("This is OK in an empty room — proceed to detection test (step 5)"))
        print(INFO("If this persists with people walking in front, check SNR thresholds"))
        return True   # not a failure — empty room is valid

    # Validate field ranges
    all_points = [p for f in frames for p in f.points]
    bad_range  = [p for p in all_points
                  if not (-10 < p.x < 10 and -10 < p.y < 10 and -5 < p.z < 5)]
    bad_vel    = [p for p in all_points if abs(p.velocity) > 30]
    bad_snr    = [p for p in all_points if p.snr < 0 or p.snr > 100]

    print(OK(f"Point cloud populated: {total_points} total points across "
             f"{len(frames_with_points)}/{len(frames)} frames"))

    if bad_range:
        print(WARN(f"{len(bad_range)} points with out-of-range XYZ "
                   f"— may indicate TLV parse issue"))
    else:
        print(OK("XYZ coordinates in valid range"))

    if bad_vel:
        print(WARN(f"{len(bad_vel)} points with velocity >30 m/s — check chirp config"))
    else:
        print(OK("Velocity values in valid range"))

    if bad_snr:
        print(WARN(f"{len(bad_snr)} points with invalid SNR"))
    else:
        print(OK("SNR values present and valid"))

    # SNR sanity
    snr_vals = [p.snr for p in all_points if p.snr > 0]
    if snr_vals:
        avg_snr = sum(snr_vals) / len(snr_vals)
        if avg_snr < 10:
            print(WARN(f"Average SNR low: {avg_snr:.1f} dB — "
                       "detections may be unreliable at range"))
        else:
            print(OK(f"Average SNR: {avg_snr:.1f} dB"))

    return True


# ── Check 5: Live detection wave test ─────────────────────────────────────

def check_live_detection(reader, timeout=20.0):
    print(STEP(5, "Live detection test — wave your hand in front of the sensor"))
    print(INFO("Stand 0.5–1.5m from the sensor and wave your hand slowly"))
    print(INFO(f"Waiting up to {timeout:.0f}s for a detection..."))
    print()

    start        = time.time()
    detected     = False
    best_snr     = 0.0
    best_range   = 0.0
    best_vel     = 0.0
    detect_count = 0

    while time.time() - start < timeout:
        frame = reader.get_frame(timeout=0.2)
        if frame is None:
            continue

        moving = [p for p in frame.points
                  if abs(p.velocity) > 0.15
                  and math.sqrt(p.x**2 + p.y**2 + p.z**2) > 0.1]

        if moving:
            detected      = True
            detect_count += 1
            best          = max(moving, key=lambda p: p.snr)
            r             = math.sqrt(best.x**2 + best.y**2 + best.z**2)
            best_snr      = max(best_snr, best.snr)
            best_range    = r
            best_vel      = best.velocity

            elapsed = time.time() - start
            print(INFO(
                f"  Detection! range={r:.2f}m  "
                f"v={best.velocity:+.2f}m/s  "
                f"SNR={best.snr:.1f}dB  "
                f"({detect_count} detections in {elapsed:.1f}s)"
            ), end='\r')

    print()

    if detected:
        print(OK(f"Live detection confirmed — "
                 f"best SNR={best_snr:.1f}dB at {best_range:.2f}m, "
                 f"v={best_vel:+.2f}m/s"))
        print(OK(f"{detect_count} detection frames in {timeout:.0f}s window"))
        return True
    else:
        print(FAIL("No detections in wave test"))
        print(INFO("Try: stand closer (0.5m), wave more vigorously"))
        print(INFO("Check: is the antenna face (heat shield) pointed at you?"))
        print(INFO("Check: profile_AOP.cfg — is startFreq 60.0 (not 77)?"))
        print(INFO("Check: is the correct firmware flashed? "
                   "(out_of_box_6843_aop.bin)"))
        return False


# ── Summary ────────────────────────────────────────────────────────────────

def print_summary(results: dict):
    print(_c("1;96", "\n" + "─" * 54))
    print(_c("1;96",   "  Validation Summary"))
    print(_c("1;96",   "─" * 54))

    all_pass = True
    for check, (passed, detail) in results.items():
        marker = OK("") if passed else FAIL("")
        status = _c("92", "PASS") if passed else _c("91", "FAIL")
        print(f"  {marker.strip()}  {check:<30} {status}")
        if detail:
            print(INFO(f"       {detail}"))
        if not passed:
            all_pass = False

    print(_c("1;96", "─" * 54))
    if all_pass:
        print(_c("92;1", """
  ✓  All checks passed — hardware is good.

  Next steps:
    Standalone pipeline:
      python3 src/main.py --dry-run --min-range 0.1 --min-velocity 0.3

    With arm controller (serial):
      python3 src/main.py --serial /dev/ttyACM0

    ROS2 pipeline:
      See README.md → ROS 2 safety node section
"""))
    else:
        print(_c("91;1", """
  ✗  One or more checks failed.
     Fix the flagged issues above before proceeding.
     See README.md → Quickstart for troubleshooting.
"""))


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="RadarGuard out-of-box hardware validation"
    )
    parser.add_argument("--cli",    default="/dev/ttyUSB0",
                        help="CLI port (default: /dev/ttyUSB0)")
    parser.add_argument("--data",   default="/dev/ttyUSB1",
                        help="Data port (default: /dev/ttyUSB1)")
    parser.add_argument("--config",
                        default=os.path.expanduser(
                            "~/mmwave/configs/profile_AOP.cfg"),
                        help="Chirp config file path")
    parser.add_argument("--no-live", action="store_true",
                        help="Skip the live wave detection test")
    args = parser.parse_args()

    banner()

    results = {}
    reader  = None

    # 1. Ports
    ports_ok = check_ports(args.cli, args.data)
    results["Port enumeration"] = (ports_ok, None)
    if not ports_ok:
        results["Config send"]       = (False, "skipped — ports not found")
        results["Frame stream"]      = (False, "skipped")
        results["Point cloud"]       = (False, "skipped")
        results["Live detection"]    = (False, "skipped")
        print_summary(results)
        sys.exit(1)

    # 2. Config
    cfg_ok, reader = check_config(args.cli, args.config)
    results["Config send"] = (
        cfg_ok,
        None if cfg_ok else "check firmware — must be out_of_box_6843_aop.bin"
    )
    if not cfg_ok or reader is None:
        results["Frame stream"]   = (False, "skipped — config failed")
        results["Point cloud"]    = (False, "skipped")
        results["Live detection"] = (False, "skipped")
        print_summary(results)
        sys.exit(1)

    # 3. Frame stream
    stream_ok, frames = check_streaming(reader, duration=5.0)
    results["Frame stream (10 Hz)"] = (
        stream_ok,
        None if stream_ok else "check data port and baud rate"
    )

    # 4. Point cloud
    if stream_ok:
        pc_ok = check_point_cloud(frames)
        results["Point cloud fields"] = (pc_ok, None)
    else:
        results["Point cloud fields"] = (False, "skipped — no frames")
        pc_ok = False

    # 5. Live detection
    if stream_ok and not args.no_live:
        det_ok = check_live_detection(reader, timeout=20.0)
        results["Live detection"] = (
            det_ok,
            None if det_ok else "check antenna orientation and firmware"
        )
    elif args.no_live:
        results["Live detection"] = (True, "skipped by user (--no-live)")
    else:
        results["Live detection"] = (False, "skipped — stream failed")

    # Cleanup
    try:
        reader.stop()
    except Exception:
        pass

    print_summary(results)
    sys.exit(0 if all(r[0] for r in results.values()) else 1)


if __name__ == "__main__":
    main()
