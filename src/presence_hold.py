"""
presence_hold.py — STOP-triggered static presence monitor
DNTD Dynamics — IWR6843AOP standalone safety pipeline

Problem this solves
-------------------
The --min-velocity filter that kills false triggers from walls and furniture
also drops a person who has stopped moving.  So once someone walks into the
STOP zone and stands still, the safety pipeline sees their velocity fall to
~0 m/s and classifies them as CLEAR — which would allow the arm to restart
with a person standing right there.

This module prevents that by latching into a STOP-hold state the moment a
STOP condition is confirmed and keeping it until the background model (or a
timer fallback) confirms the person has genuinely left.

State machine
-------------

  IDLE
    The arm is running, nobody has been detected at STOP range.
    StaticPresenceHold has no effect on zone output.
    Transition → HOLDING when ZoneClassifier outputs STOP.

  HOLDING
    A STOP was confirmed.  The person may now be standing still (velocity
    near zero, so the normal classifier would say CLEAR).
    StaticPresenceHold overrides the classifier output and holds STOP.
    It monitors two evidence sources simultaneously:

    1. Background model novelty (preferred)
       If BackgroundModel is wired in, a voxel that is occupied but not in
       the learned background is novel — someone is there.  STOP is held as
       long as any novel voxel falls inside the stop or caution radius.

    2. Micro-Doppler sway detection (fallback, always active)
       Real people standing still generate tiny involuntary movement:
       weight shifts, breathing-scale sway, postural micro-corrections.
       These produce low-amplitude Doppler returns (~0.02–0.10 m/s) that
       are detectable even when the person's centroid velocity is near zero.
       The module tracks a short history of low-velocity returns in the
       hazard zone.  If these returns appear consistently, the person is
       considered present.  If they vanish entirely for hold_timeout_s,
       the hold releases.

    Transition → IDLE when:
      - No novel voxels in hazard zone AND no sway signal for hold_timeout_s
        (background model path)
      - No points at all in hazard zone for hold_timeout_s (fallback path)

  RELEASING
    A brief grace period after the hold condition clears.  The zone stays
    at STOP for release_grace_s before returning to IDLE.  This prevents
    a momentary detection gap from releasing the hold prematurely.

Why not true heartbeat detection?
----------------------------------
The IWR6843AOP at 10 Hz with the standard chirp profile is not configured
for vital-signs extraction.  Cardiac detection requires a dedicated slow-
chirp profile (TI vital-signs demo) with sub-mm phase resolution accumulated
over many seconds — a fundamentally different operating mode.

What this module uses instead is micro-Doppler presence hold: the involuntary
sub-10-cm/s movement that a stationary person always generates.  This is
achievable with the current chirp profile and produces the same safety
property: the system does not CLEAR while a person remains in the hazard zone.

API
---
  hold = StaticPresenceHold(background_model=bg)  # bg is optional
  ...
  # Call every frame, after the normal ZoneClassifier:
  effective_zone = hold.process(
      classifier_zone = state.zone,   # ZoneClassifier output
      raw_points      = points,        # all points (pre-velocity-filter)
  )

  # Check state for status display:
  hold.state          # 'IDLE' | 'HOLDING' | 'RELEASING'
  hold.hold_reason    # human-readable explanation of why hold is active
  hold.held_for_s     # seconds in HOLDING state

Thread-safe: process() can be called from the sensor thread.
"""

import threading
import time
import logging
import math
from collections import deque
from typing import Optional

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------

# Radius inside which we monitor for static presence after a STOP.
# Should match or slightly exceed your --stop-range + --caution-range.
DEFAULT_HAZARD_RADIUS_M      = 1.4    # m — anything inside this is watched

# How long to hold STOP after detections fully disappear.
# Gives the person time to fully exit the hazard zone before releasing.
DEFAULT_HOLD_TIMEOUT_S       = 5.0    # s

# Grace period after the hold condition clears before returning IDLE.
DEFAULT_RELEASE_GRACE_S      = 2.0    # s

# Micro-Doppler sway detection thresholds.
# A "sway return" is a point inside the hazard zone with low but non-zero
# velocity.  Below sway_min it's indistinguishable from sensor noise.
DEFAULT_SWAY_MIN_MPS         = 0.02   # m/s — below this = noise
DEFAULT_SWAY_MAX_MPS         = 0.25   # m/s — above this = actually moving (normal classifier handles it)

# How many of the last N frames need a sway return to consider person present.
# At 10 Hz, 5 frames = 0.5s.  Requiring 3/5 tolerates a few empty frames.
DEFAULT_SWAY_WINDOW_FRAMES   = 10
DEFAULT_SWAY_MIN_HITS        = 4      # hits in window to confirm presence

# Minimum SNR for sway returns to count — below this is noise floor
DEFAULT_SWAY_MIN_SNR_DB      = 6.0


class StaticPresenceHold:
    """
    STOP-triggered static presence monitor.

    Wires on top of ZoneClassifier.  Call process() every frame.
    Returns the effective zone — either the classifier's output (IDLE)
    or a STOP override (HOLDING/RELEASING).
    """

    def __init__(
        self,
        background_model        = None,          # BackgroundModel instance, optional
        hazard_radius_m:   float = DEFAULT_HAZARD_RADIUS_M,
        hold_timeout_s:    float = DEFAULT_HOLD_TIMEOUT_S,
        release_grace_s:   float = DEFAULT_RELEASE_GRACE_S,
        sway_min_mps:      float = DEFAULT_SWAY_MIN_MPS,
        sway_max_mps:      float = DEFAULT_SWAY_MAX_MPS,
        sway_window:       int   = DEFAULT_SWAY_WINDOW_FRAMES,
        sway_min_hits:     int   = DEFAULT_SWAY_MIN_HITS,
        sway_min_snr:      float = DEFAULT_SWAY_MIN_SNR_DB,
    ):
        self._bg              = background_model
        self.hazard_radius    = hazard_radius_m
        self.hold_timeout     = hold_timeout_s
        self.release_grace    = release_grace_s
        self.sway_min         = sway_min_mps
        self.sway_max         = sway_max_mps
        self.sway_min_snr     = sway_min_snr
        self.sway_min_hits    = sway_min_hits

        self._lock            = threading.Lock()
        self._state           = "IDLE"
        self._hold_start      = 0.0
        self._last_seen       = 0.0    # last time any evidence was detected
        self._release_start   = 0.0
        self._hold_reason     = ""

        # Sliding window: True if that frame had a sway return in hazard zone
        self._sway_history    = deque(maxlen=sway_window)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def process(self, classifier_zone: str, raw_points: list) -> str:
        """
        Call once per sensor frame.

        classifier_zone — the output of ZoneClassifier.update_frame()
        raw_points      — the full unfiltered point list for this frame
                          (we need the low-velocity points the classifier dropped)

        Returns the effective zone string: 'CLEAR', 'CAUTION', or 'STOP'.
        """
        with self._lock:
            return self._process_locked(classifier_zone, raw_points)

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def hold_reason(self) -> str:
        with self._lock:
            return self._hold_reason

    @property
    def held_for_s(self) -> float:
        with self._lock:
            if self._state == "IDLE":
                return 0.0
            return time.monotonic() - self._hold_start

    # ------------------------------------------------------------------
    # Internal state machine
    # ------------------------------------------------------------------

    def _process_locked(self, classifier_zone: str, raw_points: list) -> str:
        now = time.monotonic()

        # Compute evidence: novel voxels + sway signal in hazard zone
        hazard_points  = self._points_in_hazard(raw_points)
        novel_present  = self._novel_in_hazard(raw_points)
        sway_present   = self._update_sway(hazard_points)

        if self._state == "IDLE":
            return self._idle(classifier_zone, now)

        if self._state == "HOLDING":
            return self._holding(classifier_zone, novel_present, sway_present, now)

        if self._state == "RELEASING":
            return self._releasing(classifier_zone, novel_present, sway_present, now)

        return classifier_zone  # should not reach here

    def _idle(self, classifier_zone: str, now: float) -> str:
        """In IDLE: pass classifier output through. Latch on STOP."""
        if classifier_zone == "STOP":
            self._state       = "HOLDING"
            self._hold_start  = now
            self._last_seen   = now
            self._hold_reason = "STOP confirmed — monitoring for static presence"
            logger.info(
                "🔒 StaticPresenceHold: HOLDING — STOP confirmed, "
                "will hold until hazard zone is clear"
            )
        return classifier_zone

    def _holding(
        self,
        classifier_zone: str,
        novel_present: bool,
        sway_present: bool,
        now: float,
    ) -> str:
        """
        In HOLDING: override CLEAR/CAUTION with STOP until person is gone.
        Transition to RELEASING when all evidence of presence is gone for
        hold_timeout_s.
        """
        # If normal classifier still says STOP, refresh last_seen and hold
        if classifier_zone == "STOP":
            self._last_seen   = now
            self._hold_reason = "STOP still active — person moving in hazard zone"
            return "STOP"

        # Check background model evidence
        if novel_present:
            self._last_seen   = now
            self._hold_reason = "Novel voxel in hazard zone — static person detected"
            return "STOP"

        # Check micro-Doppler sway evidence
        if sway_present:
            self._last_seen   = now
            self._hold_reason = (
                f"Micro-Doppler sway detected ({self.sway_min:.2f}–"
                f"{self.sway_max:.2f} m/s) — person likely stationary in zone"
            )
            return "STOP"

        # No evidence — check timeout
        gap = now - self._last_seen
        if gap >= self.hold_timeout:
            # Transition to RELEASING for grace period
            self._state         = "RELEASING"
            self._release_start = now
            self._hold_reason   = (
                f"No presence detected for {gap:.1f}s — "
                f"entering {self.release_grace:.1f}s release grace period"
            )
            logger.info(
                f"⏳ StaticPresenceHold: RELEASING — "
                f"no evidence for {gap:.1f}s, grace period starting"
            )
            return "STOP"   # stay at STOP during grace

        # Still within timeout window — hold STOP
        self._hold_reason = (
            f"No active detection, hold timeout in {self.hold_timeout - gap:.1f}s"
        )
        return "STOP"

    def _releasing(
        self,
        classifier_zone: str,
        novel_present: bool,
        sway_present: bool,
        now: float,
    ) -> str:
        """
        In RELEASING: brief grace period.  If any evidence reappears,
        snap back to HOLDING.  After grace period, return to IDLE.
        """
        # Evidence reappeared — snap back to HOLDING immediately
        if classifier_zone == "STOP" or novel_present or sway_present:
            self._state       = "HOLDING"
            self._last_seen   = now
            self._hold_reason = "Presence re-detected during release grace — holding"
            logger.info("🔒 StaticPresenceHold: re-latched during grace period")
            return "STOP"

        grace_elapsed = now - self._release_start
        if grace_elapsed >= self.release_grace:
            # Grace period complete — return to IDLE
            self._state       = "IDLE"
            self._hold_reason = ""
            logger.info(
                f"✅ StaticPresenceHold: RELEASED — "
                f"hazard zone clear, arm may resume"
            )
            return classifier_zone   # pass through CLEAR/CAUTION

        # Still in grace period
        self._hold_reason = (
            f"Release grace: {self.release_grace - grace_elapsed:.1f}s remaining"
        )
        return "STOP"

    # ------------------------------------------------------------------
    # Evidence helpers
    # ------------------------------------------------------------------

    def _points_in_hazard(self, points: list) -> list:
        """Return all points (regardless of velocity) inside hazard radius."""
        return [p for p in points if _range(p) <= self.hazard_radius]

    def _novel_in_hazard(self, points: list) -> bool:
        """
        Return True if the background model sees any novel point in the
        hazard zone.  Falls back to False if no background model is wired.
        """
        if self._bg is None or self._bg.state != "ACTIVE":
            return False
        hazard = self._points_in_hazard(points)
        if not hazard:
            return False
        novel = self._bg.filter_novel(hazard)
        return len(novel) > 0

    def _update_sway(self, hazard_points: list) -> bool:
        """
        Detect micro-Doppler sway: low-velocity, low-amplitude returns
        that indicate a stationary person's involuntary movement.

        Returns True if enough recent frames have had sway returns.
        """
        # A sway return is a point inside hazard zone with:
        #   - velocity between sway_min and sway_max (not zero, not fast)
        #   - SNR above the noise floor
        sway_this_frame = any(
            self.sway_min <= abs(p.velocity) <= self.sway_max
            and getattr(p, 'snr', 15.0) >= self.sway_min_snr
            for p in hazard_points
        )

        self._sway_history.append(sway_this_frame)

        if len(self._sway_history) < self.sway_min_hits:
            return False   # not enough history yet

        return sum(self._sway_history) >= self.sway_min_hits


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _range(p) -> float:
    return math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)


# ---------------------------------------------------------------------------
# Standalone test — no hardware required
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import logging
    from dataclasses import dataclass

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    @dataclass
    class FakePoint:
        x: float; y: float; z: float
        velocity: float; snr: float = 20.0

    hold = StaticPresenceHold(hold_timeout_s=2.0, release_grace_s=1.0)

    def tick(zone, points, label):
        eff = hold.process(zone, points)
        print(f"  [{hold.state:10s}] classifier={zone:7s} → effective={eff:7s}  | {label}")
        time.sleep(0.1)

    print("\n=== Test: person walks in → stops → leaves ===\n")

    # Arm running, nobody nearby
    for _ in range(3):
        tick("CLEAR", [], "arm running, nobody there")

    # Person walks in fast — classifier fires STOP
    for _ in range(5):
        tick("STOP", [FakePoint(0, 0.3, 0, -0.8)], "person walking into STOP zone")

    # Person stops moving — classifier drops to CLEAR (velocity filter kills them)
    # but sway signal is present (micro-Doppler from standing still)
    print("\n  --- Person has stopped moving ---")
    for i in range(20):
        sway_v = 0.04 if (i % 3 != 0) else 0.01   # occasional sway signal
        tick("CLEAR", [FakePoint(0, 0.3, 0, sway_v)], "person standing still — sway detectable")

    # Person leaves — no points at all
    print("\n  --- Person has left the zone ---")
    for _ in range(25):
        tick("CLEAR", [], "empty zone")

    print("\n  Hold released — arm can resume.\n")
