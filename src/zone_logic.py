"""
zone_logic.py — mmWave safety zone classifier for arm-mounted sensor
DNTD Dynamics — IWR6843AOP on 6-DOF robot arm

Zone classification is based on BOTH range and approach velocity,
so fast-moving objects (falling person, sudden reach-in) trigger
STOP regardless of current distance.

Output interface (all simultaneous, pick what your controller uses):
  - zone_state dict  → poll from main.py
  - Serial string    → any UART-capable board (Arduino, Pi, Jetson)
  - GPIO pin         → 3 pins, logic HIGH = active (3.3V or 5V)
  - MQTT topic       → mmwave/zone_state (optional, needs paho-mqtt)

Zone definitions (sensor-relative, meters):
  CLEAR   — nothing within range, or only slow/static returns
  CAUTION — object within CAUTION_RANGE, arm slows down
  STOP    — object within STOP_RANGE, OR fast approach from CAUTION zone

Tunable constants are at the top — adjust per arm geometry.
"""

import time
import threading
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Zone geometry — tune these to match your arm's physical reach
# ---------------------------------------------------------------------------
STOP_RANGE_M        = 0.5    # m  — hard stop: anything inside this
CAUTION_RANGE_M     = 1.2    # m  — slow down: anything inside this
CLEAR_RANGE_M       = 1.2    # m  — beyond this = clear (same as CAUTION outer edge)

# Velocity thresholds — sensor-relative (negative = approaching)
# A point approaching faster than this triggers STOP even from CAUTION zone
FAST_APPROACH_MPS   = -0.8   # m/s — ~3 km/h, a brisk step or stumble
STATIC_FILTER_MPS   = 0.3    # m/s — below this magnitude = static clutter, ignored

# Minimum SNR to consider a detection valid
MIN_SNR_DB          = 8.0    # dB  — filters noise, keep real detections

# Hysteresis: how many consecutive frames before zone transitions
# Prevents oscillation at zone boundaries
HYSTERESIS_FRAMES   = 2      # frames to confirm a zone upgrade (CLEAR→CAUTION→STOP)
CLEAR_HYSTERESIS    = 4      # frames of CLEAR before downgrading from STOP/CAUTION

# Frame rate from sensor (used for timing only, not hard-coded into logic)
SENSOR_HZ           = 10


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
@dataclass
class DetectedPoint:
    x: float          # m, lateral (positive = right of sensor face)
    y: float          # m, range (positive = in front of sensor)
    z: float          # m, elevation (positive = above)
    velocity: float   # m/s, sensor-relative (negative = approaching)
    snr: float        # dB


@dataclass
class ZoneState:
    zone: str                    # 'CLEAR', 'CAUTION', 'STOP'
    reason: str                  # human-readable explanation
    closest_m: Optional[float]   # range of closest valid detection
    fastest_approach_mps: Optional[float]   # most negative velocity seen
    point_count: int             # number of valid detections this frame
    timestamp: float = field(default_factory=time.time)

    def __str__(self):
        closest  = f"{self.closest_m:.2f}m"  if self.closest_m  is not None else "—"
        approach = f"{self.fastest_approach_mps:.2f}m/s" if self.fastest_approach_mps is not None else "—"
        return (f"[{self.zone}] {self.reason} | "
                f"closest={closest} | approach={approach} | pts={self.point_count}")


# ---------------------------------------------------------------------------
# Zone classifier
# ---------------------------------------------------------------------------
class ZoneClassifier:
    """
    Classifies each frame of point cloud data into CLEAR / CAUTION / STOP.

    Thread-safe: call update_frame() from your reader thread,
    call get_state() from your control loop.
    """

    def __init__(
        self,
        stop_range:       float = STOP_RANGE_M,
        caution_range:    float = CAUTION_RANGE_M,
        fast_approach:    float = FAST_APPROACH_MPS,
        static_filter:    float = STATIC_FILTER_MPS,
        min_snr:          float = MIN_SNR_DB,
        hysteresis:       int   = HYSTERESIS_FRAMES,
        clear_hysteresis: int   = CLEAR_HYSTERESIS,
    ):
        self.stop_range       = stop_range
        self.caution_range    = caution_range
        self.fast_approach    = fast_approach
        self.static_filter    = static_filter
        self.min_snr          = min_snr
        self.hysteresis       = hysteresis
        self.clear_hysteresis = clear_hysteresis

        self._lock            = threading.Lock()
        self._current_zone    = "CLEAR"
        self._state           = ZoneState("CLEAR", "Initializing", None, None, 0)

        # Ring buffers for hysteresis
        self._raw_zone_history = deque(maxlen=max(hysteresis, clear_hysteresis))

    # ------------------------------------------------------------------
    def update_frame(self, points: list[DetectedPoint]) -> ZoneState:
        """
        Call once per sensor frame with the decoded point list.
        Returns the current ZoneState (after hysteresis).
        """
        raw_zone, reason, closest, fastest, count = self._classify_points(points)

        with self._lock:
            self._raw_zone_history.append(raw_zone)
            confirmed_zone = self._apply_hysteresis(raw_zone)
            self._current_zone = confirmed_zone
            self._state = ZoneState(
                zone=confirmed_zone,
                reason=reason,
                closest_m=closest,
                fastest_approach_mps=fastest,
                point_count=count,
            )
            return self._state

    def get_state(self) -> ZoneState:
        with self._lock:
            return self._state

    # ------------------------------------------------------------------
    def _classify_points(self, points):
        """
        Core classification logic. Returns (zone, reason, closest, fastest, count).
        No hysteresis here — raw per-frame decision only.

        Two-stage filter applied before classification:
          1. SNR gate  — drops noise returns below min_snr threshold
          2. Velocity gate (static_filter) — drops near-static returns
             (walls, fixtures, mount hardware at v ≈ 0).
             Note: during STOP-triggered presence hold, this filter is
             intentionally bypassed by the caller injecting pre-filtered
             points — see StaticPresenceHold in main.py.
        """
        # SNR gate
        snr_valid = [p for p in points if p.snr >= self.min_snr]

        # Velocity gate — the static_filter was stored but never applied here
        # (the bug: only main.py's --min-velocity pre-pass was filtering).
        # Now it gates here too, so the classifier always enforces it regardless
        # of how main.py is called.
        valid = [
            p for p in snr_valid
            if abs(p.velocity) >= self.static_filter
        ]

        if not valid:
            return "CLEAR", "No valid detections", None, None, 0

        # Compute range (distance from sensor) for each valid point
        ranges = [_range(p) for p in valid]
        velocities = [p.velocity for p in valid]

        closest      = min(ranges)
        fastest      = min(velocities)   # most negative = fastest approach
        count        = len(valid)

        # --- STOP conditions ---
        # 1. Anything inside hard stop radius
        if closest <= self.stop_range:
            return (
                "STOP",
                f"Object at {closest:.2f}m (≤ stop threshold {self.stop_range}m)",
                closest, fastest, count,
            )

        # 2. Fast approach from caution zone (stumble/fall detection)
        if closest <= self.caution_range and fastest <= self.fast_approach:
            return (
                "STOP",
                f"Fast approach {fastest:.2f}m/s at {closest:.2f}m — emergency stop",
                closest, fastest, count,
            )

        # --- CAUTION condition ---
        if closest <= self.caution_range:
            return (
                "CAUTION",
                f"Object at {closest:.2f}m (≤ caution threshold {self.caution_range}m)",
                closest, fastest, count,
            )

        # --- CLEAR ---
        return (
            "CLEAR",
            f"Closest valid object at {closest:.2f}m",
            closest, fastest, count,
        )

    # ------------------------------------------------------------------
    def _apply_hysteresis(self, raw_zone: str) -> str:
        """
        Upgrades (CLEAR→CAUTION→STOP) happen quickly (hysteresis frames).
        Downgrades (STOP→CAUTION→CLEAR) are slower (clear_hysteresis frames).

        This prevents oscillation at zone boundaries and ensures the arm
        doesn't resume before the person is clearly gone.
        """
        history = list(self._raw_zone_history)
        if not history:
            return raw_zone

        current = self._current_zone

        # Upgrading: check if recent frames consistently show a worse zone
        # Use only the last `hysteresis` frames for upgrade decisions
        upgrade_window = history[-self.hysteresis:]
        if all(z == "STOP" for z in upgrade_window):
            return "STOP"
        if all(z in ("STOP", "CAUTION") for z in upgrade_window):
            return "CAUTION" if current == "CLEAR" else current

        # Immediate upgrade — never delay a STOP trigger
        if raw_zone == "STOP":
            return "STOP"
        if raw_zone == "CAUTION" and current == "CLEAR":
            return "CAUTION"

        # Downgrading: require clear_hysteresis consecutive CLEAR frames
        clear_window = history[-self.clear_hysteresis:]
        if len(clear_window) >= self.clear_hysteresis:
            if all(z == "CLEAR" for z in clear_window):
                return "CLEAR"
            if all(z in ("CLEAR", "CAUTION") for z in clear_window) and current == "STOP":
                return "CAUTION"

        return current


# ---------------------------------------------------------------------------
# Output interface — serial, GPIO, MQTT
# ---------------------------------------------------------------------------
class ZoneOutputs:
    """
    Publishes zone state to whichever outputs are available.
    All outputs are optional — pass None to skip.

    Serial protocol (simple, Arduino/Pi/Jetson compatible):
        "<ZONE>\\n"  →  "CLEAR\\n" / "CAUTION\\n" / "STOP\\n"

    GPIO pin map (BCM numbering, Raspberry Pi default):
        PIN_CLEAR   HIGH when zone == CLEAR
        PIN_CAUTION HIGH when zone == CAUTION
        PIN_STOP    HIGH when zone == STOP

    MQTT topic:
        mmwave/zone_state  →  "CLEAR" / "CAUTION" / "STOP"
        mmwave/zone_detail →  JSON with full ZoneState fields
    """

    # GPIO BCM pin numbers — change to match your wiring
    PIN_CLEAR   = 17
    PIN_CAUTION = 27
    PIN_STOP    = 22

    def __init__(
        self,
        serial_port:   Optional[str] = None,   # e.g. "/dev/ttyACM0" or "COM3"
        serial_baud:   int            = 115200,
        use_gpio:      bool           = False,
        mqtt_broker:   Optional[str]  = None,   # e.g. "192.168.254.117"
        mqtt_port:     int            = 1883,
        mqtt_topic:    str            = "mmwave/zone_state",
    ):
        self._last_zone  = None
        self._serial     = None
        self._gpio       = None
        self._mqtt       = None
        self._mqtt_topic = mqtt_topic

        # Serial
        if serial_port:
            try:
                import serial
                self._serial = serial.Serial(serial_port, serial_baud, timeout=1)
                logger.info(f"Zone output: serial on {serial_port} @ {serial_baud}")
            except Exception as e:
                logger.warning(f"Serial output unavailable: {e}")

        # GPIO (RPi only — gracefully skipped on non-Pi hardware)
        if use_gpio:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                for pin in (self.PIN_CLEAR, self.PIN_CAUTION, self.PIN_STOP):
                    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
                self._gpio = GPIO
                logger.info(f"Zone output: GPIO pins {self.PIN_CLEAR}/{self.PIN_CAUTION}/{self.PIN_STOP}")
            except Exception as e:
                logger.warning(f"GPIO output unavailable (non-Pi?): {e}")

        # MQTT
        if mqtt_broker:
            try:
                import paho.mqtt.client as mqtt
                self._mqtt = mqtt.Client()
                self._mqtt.connect(mqtt_broker, mqtt_port, keepalive=60)
                self._mqtt.loop_start()
                self._mqtt_topic = mqtt_topic
                logger.info(f"Zone output: MQTT → {mqtt_broker}:{mqtt_port}/{mqtt_topic}")
            except Exception as e:
                logger.warning(f"MQTT output unavailable: {e}")

    # ------------------------------------------------------------------
    def publish(self, state: ZoneState):
        """Publish zone state to all configured outputs. Only on zone change."""
        if state.zone == self._last_zone:
            return
        self._last_zone = state.zone

        logger.info(str(state))

        if self._serial:
            try:
                self._serial.write(f"{state.zone}\n".encode())
            except Exception as e:
                logger.warning(f"Serial write failed: {e}")

        if self._gpio:
            self._set_gpio(state.zone)

        if self._mqtt:
            import json
            try:
                self._mqtt.publish(self._mqtt_topic, state.zone)
                self._mqtt.publish(
                    self._mqtt_topic.replace("zone_state", "zone_detail"),
                    json.dumps({
                        "zone":     state.zone,
                        "reason":   state.reason,
                        "closest":  state.closest_m,
                        "approach": state.fastest_approach_mps,
                        "points":   state.point_count,
                        "ts":       state.timestamp,
                    })
                )
            except Exception as e:
                logger.warning(f"MQTT publish failed: {e}")

    def _set_gpio(self, zone: str):
        GPIO = self._gpio
        GPIO.output(self.PIN_CLEAR,   zone == "CLEAR")
        GPIO.output(self.PIN_CAUTION, zone == "CAUTION")
        GPIO.output(self.PIN_STOP,    zone == "STOP")

    def cleanup(self):
        if self._serial:
            self._serial.close()
        if self._gpio:
            self._gpio.cleanup()
        if self._mqtt:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _range(p: DetectedPoint) -> float:
    """3D Euclidean distance from sensor origin."""
    return (p.x**2 + p.y**2 + p.z**2) ** 0.5


def points_from_tlv_frame(frame: dict) -> list[DetectedPoint]:
    """
    Converts a decoded TLV frame dict (as produced by tlv_parser.py)
    into a list of DetectedPoint objects.

    Expected frame format:
        {
          "detected_points": [
            {"x": float, "y": float, "z": float, "velocity": float},
            ...
          ],
          "side_info": [
            {"snr": float, "noise": float},   # optional, index-matched
            ...
          ]
        }
    """
    points = []
    raw_pts  = frame.get("detected_points", [])
    side_info = frame.get("side_info", [])

    for i, pt in enumerate(raw_pts):
        snr = side_info[i]["snr"] if i < len(side_info) else 15.0  # assume valid if no side info
        points.append(DetectedPoint(
            x        = pt["x"],
            y        = pt["y"],
            z        = pt["z"],
            velocity = pt["velocity"],
            snr      = snr,
        ))
    return points


# ---------------------------------------------------------------------------
# Standalone test — runs against mock data without hardware
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    classifier = ZoneClassifier()

    test_frames = [
        # (description, points)
        ("Nothing detected",
         []),
        ("Person at 2.0m, slow walk",
         [DetectedPoint(0.1, 2.0, 0.0, -0.3, 20.0)]),
        ("Person at 1.0m, normal walk",
         [DetectedPoint(0.1, 1.0, 0.0, -0.6, 25.0)]),
        ("Person at 0.8m, approaching at moderate speed",
         [DetectedPoint(0.0, 0.8, 0.0, -0.9, 28.0)]),   # fast approach → STOP
        ("Person at 0.3m, inside stop zone",
         [DetectedPoint(0.0, 0.3, 0.0, -0.4, 30.0)]),   # range → STOP
        ("Person retreating to 1.5m",
         [DetectedPoint(0.1, 1.5, 0.0, +0.6, 20.0)]),
        ("Same — confirming clear (hysteresis)",
         [DetectedPoint(0.1, 1.5, 0.0, +0.6, 20.0)]),
        ("Same — confirming clear (hysteresis)",
         [DetectedPoint(0.1, 1.5, 0.0, +0.6, 20.0)]),
        ("Same — confirming clear (hysteresis)",
         [DetectedPoint(0.1, 1.5, 0.0, +0.6, 20.0)]),
        ("Room empty",
         []),
    ]

    print(f"\n{'─'*70}")
    print(f"  Zone classifier test — DNTD Dynamics mmWave safety layer")
    print(f"  stop={STOP_RANGE_M}m  caution={CAUTION_RANGE_M}m  fast_approach={FAST_APPROACH_MPS}m/s")
    print(f"{'─'*70}\n")

    for desc, pts in test_frames:
        state = classifier.update_frame(pts)
        marker = {"CLEAR": "✅", "CAUTION": "⚠️ ", "STOP": "🛑"}[state.zone]
        print(f"{marker}  {desc}")
        print(f"    → {state}\n")
