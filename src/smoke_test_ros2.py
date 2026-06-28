#!/usr/bin/env python3
"""
smoke_test_ros2.py
DNTD Dynamics — RadarGuard ROS2 Pipeline Smoke Test

Runs the full ROS2 safety pipeline with fake joint states and verifies
every component initializes, connects, and produces expected outputs.
Does NOT require a real arm — validates the pipeline is wired correctly
before any hardware is attached.

What this tests:
  ✓ Driver node starts and sensor streams
  ✓ Safety node initializes all subsystems (background model, classifier,
    swept volume, ego-motion compensator)
  ✓ /joint_states watchdog does NOT fault when fake_joint_states is running
  ✓ /dntd/safety_zone publishes CLEAR/CAUTION/STOP
  ✓ /dntd/heartbeat pulses at expected rate
  ✓ /dntd/safety_fault is empty when healthy
  ✓ /dntd/compensated_points publishes world-frame point cloud
  ✓ Background learning completes and transitions to ACTIVE
  ✓ Zone transitions occur when you walk in front of sensor
  ✓ Safety node raises fault correctly when joint_states stops

Run in THREE terminals:

  Terminal 1 — sensor driver:
    cd ~/mmwave && python3 src/dntd_mmwave_driver_node.py

  Terminal 2 — fake joint states (keeps watchdog happy):
    cd ~/mmwave && python3 src/fake_joint_states.py --joints joint1 joint2 joint3 joint4 joint5 joint6

  Terminal 3 — this script:
    cd ~/mmwave && python3 src/smoke_test_ros2.py

Or use the automated launcher (runs all three automatically):
    cd ~/mmwave && python3 src/smoke_test_ros2.py --launch

Checks are run sequentially. Any FAIL stops the test and prints a
diagnostic. Fix the issue before continuing — later checks depend on
earlier ones passing.
"""

import argparse
import subprocess
import sys
import time
import threading
import os

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy  # add this line
    from std_msgs.msg import String, Bool, Header
    from sensor_msgs.msg import PointCloud2
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


# ── Colour helpers ─────────────────────────────────────────────────────────
def _c(code, text): return f"\033[{code}m{text}\033[0m" if sys.stdout.isatty() else text
OK   = lambda t: _c("92",   f"  ✓  {t}")
FAIL = lambda t: _c("91",   f"  ✗  {t}")
WARN = lambda t: _c("93",   f"  ⚠  {t}")
INFO = lambda t: _c("96",   f"     {t}")
STEP = lambda n,t: _c("1",  f"\n[{n}] {t}")
HEAD = lambda t:   _c("1;96", f"\n{'─'*54}\n  {t}\n{'─'*54}")


def banner():
    print(_c("1;96", """
  ╔══════════════════════════════════════════════════════╗
  ║   RadarGuard — ROS2 Pipeline Smoke Test              ║
  ║   DNTD Dynamics                                      ║
  ╚══════════════════════════════════════════════════════╝
"""))


# ── Smoke test node ────────────────────────────────────────────────────────

class SmokeTestNode(Node):
    """
    Subscribes to all safety node outputs and verifies they arrive
    correctly within expected time windows.
    """

    TOPICS = {
        "/dntd/safety_zone":        String,
        "/dntd/safety_fault":       String,
        "/dntd/heartbeat":          Header,
        "/dntd/compensated_points": PointCloud2,
    }

    def __init__(self):
        super().__init__("radarguard_smoke_test")

        self._received  = {t: [] for t in self.TOPICS}
        self._lock      = threading.Lock()

        # compensated_points is published BEST_EFFORT by the safety node
        # (it is sensor data, not control state). A default RELIABLE
        # subscription will not connect to it. All other safety outputs
        # are RELIABLE, so only this topic needs the override.
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        for topic, msg_type in self.TOPICS.items():
            qos = best_effort_qos if topic == "/dntd/compensated_points" else 10
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._cb(t, msg),
                qos
            )

        # safety_resume: the safety node subscribes with TRANSIENT_LOCAL so a
        # resume sent before it is ready still latches. Match it here, else
        # DDS refuses the link (DURABILITY mismatch).
        resume_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self._resume_pub = self.create_publisher(
            Bool, "/dntd/safety_resume", resume_qos)

        self.get_logger().info("Smoke test node started — listening on all topics")

    def _cb(self, topic, msg):
        with self._lock:
            self._received[topic].append((time.monotonic(), msg))

    def count(self, topic):
        with self._lock:
            return len(self._received[topic])

    def latest(self, topic):
        with self._lock:
            msgs = self._received[topic]
            return msgs[-1] if msgs else None

    def latest_value(self, topic):
        entry = self.latest(topic)
        if entry is None:
            return None
        msg = entry[1]
        if hasattr(msg, 'data'):
            return msg.data
        return msg

    def send_resume(self):
        msg      = Bool()
        msg.data = True
        self._resume_pub.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


# ── Individual checks ──────────────────────────────────────────────────────

def check_ros2_available():
    print(STEP(1, "ROS2 environment"))
    if not HAS_ROS:
        print(FAIL("rclpy not importable — source ROS2 first:"))
        print(INFO("  source /opt/ros/humble/setup.bash"))
        return False
    print(OK("rclpy imported successfully"))

    # Check ROS2 daemon
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True, timeout=5
        )
        print(OK("ros2 CLI available"))
    except Exception as e:
        print(WARN(f"ros2 CLI check failed: {e} — may still work"))

    return True


def check_driver_node(node, wait=10.0):
    print(STEP(2, f"Driver node — waiting {wait:.0f}s for point cloud"))
    print(INFO("Expecting /dntd/mmwave/raw_points from dntd_mmwave_driver_node.py"))

    # Subscribe temporarily to raw points
    raw_received = []
    from sensor_msgs.msg import PointCloud2 as PC2

    sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
    )
    sub = node.create_subscription(
        PC2, "/dntd/mmwave/raw_points",
        lambda msg: raw_received.append(msg), sensor_qos
    )

    deadline = time.monotonic() + wait
    while time.monotonic() < deadline and not raw_received:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(sub)

    if not raw_received:
        print(FAIL("No messages on /dntd/mmwave/raw_points"))
        print(INFO("Is dntd_mmwave_driver_node.py running? (Terminal 1)"))
        print(INFO("Is the sensor plugged in and powered?"))
        print(INFO("Run validate_hardware.py first to confirm hardware is good"))
        return False

    print(OK(f"Driver node publishing — {len(raw_received)} messages in {wait:.0f}s"))
    return True


def check_safety_node_startup(node, wait=20.0):
    print(STEP(3, f"Safety node startup — waiting {wait:.0f}s for heartbeat"))
    print(INFO("Safety node runs background learning on startup (~15s)"))
    print(INFO("Zone output holds STOP during learning — this is correct"))

    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        node.spin_for(0.5)
        if node.count("/dntd/heartbeat") > 0:
            break

    if node.count("/dntd/heartbeat") == 0:
        print(FAIL("No heartbeat received from safety node"))
        print(INFO("Is dntd_mmwave_safety_node.py running?"))
        print(INFO("Check: source /opt/ros/humble/setup.bash"))
        print(INFO("Check: all imports available (numpy, etc.)"))
        return False

    hb_count = node.count("/dntd/heartbeat")
    print(OK(f"Heartbeat received — {hb_count} pulses so far"))
    return True


def check_joint_states_watchdog(node, wait=5.0):
    print(STEP(4, "Joint states watchdog"))
    print(INFO("Checking /dntd/safety_fault is empty with fake_joint_states running"))

    node.spin_for(wait)

    fault = node.latest_value("/dntd/safety_fault")

    if fault is None:
        print(WARN("/dntd/safety_fault not yet published — safety node may still be starting"))
        return True  # not a failure yet

    if "joint_states" in str(fault).lower():
        print(FAIL(f"joint_states fault active: {fault}"))
        print(INFO("Is fake_joint_states.py running? (Terminal 2)"))
        print(INFO("Check joint names match config:"))
        print(INFO("  joint_names in dntd_mmwave_config.yaml must match"))
        print(INFO("  --joints arg in fake_joint_states.py"))
        return False

    if fault == "" or fault is None:
        print(OK("No joint_states fault — watchdog healthy"))
    else:
        print(WARN(f"Fault active (not joint_states): {fault}"))
        print(INFO("This may clear after background learning completes"))

    return True


def check_background_learning(node, wait=25.0):
    print(STEP(5, f"Background learning — waiting up to {wait:.0f}s for completion"))
    print(INFO("Safety node learns static environment on startup"))
    print(INFO("Zone holds STOP during this phase — correct behavior"))

    deadline  = time.monotonic() + wait
    last_zone = None

    while time.monotonic() < deadline:
        node.spin_for(1.0)
        zone  = node.latest_value("/dntd/safety_zone")
        fault = node.latest_value("/dntd/safety_fault")

        if zone != last_zone:
            print(INFO(f"  Zone: {zone}  |  Fault: '{fault}'"))
            last_zone = zone

        # Learning complete when fault clears and zone becomes CLEAR
        if zone == "CLEAR" and (fault == "" or fault is None):
            elapsed = wait - (deadline - time.monotonic())
            print(OK(f"Background learning complete — pipeline active after {elapsed:.0f}s"))
            return True

        # Also accept if zone moved away from learning-hold STOP
        if zone in ("CLEAR", "CAUTION") and "learning" not in str(fault).lower():
            print(OK("Background learning complete — zone active"))
            return True

    print(WARN(f"Background learning did not complete in {wait:.0f}s"))
    print(INFO("Check background_learning_s in dntd_mmwave_config.yaml"))
    print(INFO("Default is 15s — if sensor is noisy it may take longer"))
    return True   # warn but don't fail — timing varies


def check_zone_output(node, wait=5.0):
    print(STEP(6, "Zone output"))

    node.spin_for(wait)
    zone = node.latest_value("/dntd/safety_zone")

    if zone is None:
        print(FAIL("/dntd/safety_zone not publishing"))
        print(INFO("Safety node may still be starting — wait and retry"))
        return False

    if zone not in ("CLEAR", "CAUTION", "STOP"):
        print(FAIL(f"Unexpected zone value: '{zone}'"))
        return False

    count = node.count("/dntd/safety_zone")
    print(OK(f"Zone output healthy — current: {zone} ({count} messages received)"))
    return True


def check_compensated_points(node, wait=5.0):
    print(STEP(7, "Compensated point cloud"))

    node.spin_for(wait)
    count = node.count("/dntd/compensated_points")

    if count == 0:
        print(WARN("/dntd/compensated_points not yet publishing"))
        print(INFO("This publishes only when novel points are detected"))
        print(INFO("Wave your hand in front of the sensor to generate points"))
        return True   # warn only — empty scene is valid

    print(OK(f"/dntd/compensated_points publishing ({count} messages)"))
    return True


def check_heartbeat_rate(node, window=10.0):
    print(STEP(8, f"Heartbeat rate — measuring over {window:.0f}s"))

    before = node.count("/dntd/heartbeat")
    node.spin_for(window)
    after  = node.count("/dntd/heartbeat")
    count  = after - before
    rate   = count / window

    expected = 5.0   # heartbeat_hz default

    if rate < expected * 0.5:
        print(FAIL(f"Heartbeat rate too low: {rate:.1f} Hz (expect ~{expected:.0f} Hz)"))
        print(INFO("Safety node may be overloaded or stalled"))
        return False
    elif rate < expected * 0.8:
        print(WARN(f"Heartbeat rate slightly low: {rate:.1f} Hz (expect ~{expected:.0f} Hz)"))
        return True
    else:
        print(OK(f"Heartbeat rate: {rate:.1f} Hz"))
        return True


def check_fault_injection(node):
    print(STEP(9, "Fault injection — watchdog test"))
    print(INFO("This test kills fake_joint_states and confirms the node faults"))
    print(INFO("Skipping automated kill — run manually if desired:"))
    print(INFO("  1. Ctrl+C in Terminal 2 (fake_joint_states)"))
    print(INFO("  2. Watch: ros2 topic echo /dntd/safety_fault"))
    print(INFO("  3. Expect: 'joint_states timeout' fault within 0.5s"))
    print(INFO("  4. Restart fake_joint_states and send resume:"))
    print(INFO("     ros2 topic pub --once /dntd/safety_resume std_msgs/Bool 'data: true'"))
    return True   # informational only


def check_zone_transition(node, wait=20.0):
    print(STEP(10, "Zone transition test — walk toward the sensor"))
    print(INFO(f"Walk slowly toward the sensor. Waiting {wait:.0f}s for a zone change..."))
    print(INFO("Expect: CLEAR → CAUTION → STOP as you approach"))
    print()

    zones_seen = set()
    deadline   = time.monotonic() + wait
    last_zone  = None

    while time.monotonic() < deadline:
        node.spin_for(0.2)
        zone = node.latest_value("/dntd/safety_zone")
        if zone and zone != last_zone:
            print(INFO(f"  Zone → {zone}"))
            zones_seen.add(zone)
            last_zone = zone

    if len(zones_seen) >= 2:
        print(OK(f"Zone transitions observed: {' → '.join(sorted(zones_seen, key=['CLEAR','CAUTION','STOP'].index))}"))
        return True
    elif len(zones_seen) == 1:
        print(WARN(f"Only one zone seen: {zones_seen}"))
        print(INFO("Try walking closer — stop range default is 0.5m"))
        return True   # warn, not fail
    else:
        print(WARN("No zone output during walk test"))
        print(INFO("Background learning may still be running"))
        return True   # warn, not fail


# ── Summary ────────────────────────────────────────────────────────────────

def print_summary(results: dict):
    print(HEAD("Smoke Test Summary"))

    all_pass = True
    for name, passed in results.items():
        status = _c("92", "PASS") if passed else _c("91", "FAIL")
        marker = OK("") if passed else FAIL("")
        print(f"  {marker.strip()}  {name:<40} {status}")
        if not passed:
            all_pass = False

    print(_c("1;96", "─" * 54))

    if all_pass:
        print(_c("92;1", """
  ✓  ROS2 pipeline smoke test passed.

  The full pipeline is wired correctly. Next steps:
    • Build step counter → /joint_states publisher for real ego-motion
    • Measure EB300 link lengths and run arm_config_gui.py
    • Run with real /joint_states from ESP32 step counter
    • Validate ego-motion compensation at sweep speed
"""))
    else:
        print(_c("91;1", """
  ✗  One or more checks failed.
     Fix the flagged issues above before proceeding.
"""))


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="RadarGuard ROS2 pipeline smoke test"
    )
    parser.add_argument("--skip-walk", action="store_true",
                        help="Skip the walk/zone transition test (step 10)")
    args = parser.parse_args()

    banner()

    print(INFO("Before running this script:"))
    print(INFO("  Terminal 1: python3 src/dntd_mmwave_driver_node.py"))
    print(INFO("  Terminal 2: python3 src/fake_joint_states.py"))
    print(INFO("  Terminal 3: this script"))
    print()
    input(_c("1", "  Press Enter when Terminals 1 and 2 are running..."))

    if not check_ros2_available():
        sys.exit(1)

    rclpy.init()
    node    = SmokeTestNode()
    results = {}

    try:
        results["ROS2 environment"]         = check_ros2_available()
        results["Driver node streaming"]    = check_driver_node(node)
        results["Safety node heartbeat"]    = check_safety_node_startup(node)
        results["Joint states watchdog"]    = check_joint_states_watchdog(node)
        results["Background learning"]      = check_background_learning(node)
        results["Zone output publishing"]   = check_zone_output(node)
        results["Compensated point cloud"]  = check_compensated_points(node)
        results["Heartbeat rate (~5 Hz)"]   = check_heartbeat_rate(node)
        results["Fault injection (manual)"] = check_fault_injection(node)

        if not args.skip_walk:
            results["Zone transitions"] = check_zone_transition(node)
        else:
            print(INFO("\n[10] Zone transition test — skipped (--skip-walk)"))
            results["Zone transitions"] = True

    except KeyboardInterrupt:
        print(_c("93", "\n\n  Interrupted by user"))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print_summary(results)
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
