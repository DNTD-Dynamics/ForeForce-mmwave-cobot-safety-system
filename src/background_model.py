"""
background_model.py
DNTD Dynamics — mmWave Background Scene Model

Learns the static environment (walls, fixtures, mounts) over time and
masks those returns so the safety system only reacts to novel objects
(people, tools, anything not present during learning).

Algorithm:
  1. Discretize 3D space into voxels (default 10cm cubes)
  2. For each frame, increment a counter for every voxel that has a return
  3. After learning_duration_s, freeze voxels with high counters as "background"
  4. After learning ends, presence in a non-background voxel = NOVEL detection
  5. Background voxels slowly decay if not refreshed (handles moved furniture)
  6. Voxels refresh their counter when they continue to see returns

State machine:
  LEARNING   → counters accumulate, no detections published
  ACTIVE     → masking enabled, novel returns flagged
  RELEARNING → counters reset, briefly LEARNING again

Thread-safe — call observe() from sensor thread, is_novel() from anywhere.
"""

import math
import threading
import time
import logging
from collections import defaultdict
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class BackgroundModelStats:
    """Snapshot of model state for diagnostics."""
    state:               str       # LEARNING | ACTIVE | RELEARNING
    seconds_remaining:   float     # in learning phase, 0 once ACTIVE
    voxel_count:         int       # total voxels with any observations
    background_voxels:   int       # voxels classified as background
    frames_observed:     int       # total frames seen since model start
    decay_events:        int       # how many voxels have decayed below threshold


class BackgroundModel:
    """
    Voxel-grid background learner.

    Public API:
      observe(points)        — call every frame with the raw point list
      is_novel(point)        — query whether a point is novel (not background)
      filter_novel(points)   — return only the novel points from a list
      start_relearn()        — clear and re-enter LEARNING state
      get_stats()            — diagnostic snapshot
    """

    # ------------------------------------------------------------------
    # Tunable constants (all overridable via constructor)
    # ------------------------------------------------------------------
    DEFAULT_VOXEL_SIZE      = 0.10   # m — voxel edge length
    DEFAULT_LEARNING_TIME   = 15.0   # s — initial learning duration
    DEFAULT_HIT_THRESHOLD   = 0.30   # fraction of frames a voxel must see returns
                                     # during learning to count as background
    DEFAULT_DECAY_RATE      = 0.001  # per-frame counter decrement when not seen
    DEFAULT_REFRESH_RATE    = 0.020  # per-frame counter increment when seen
    DEFAULT_BACKGROUND_THR  = 0.50   # voxel "background_score" threshold post-learning
    DEFAULT_MAX_RANGE       = 8.92   # m — match radar max range from .cfg

    def __init__(
        self,
        voxel_size:           float = DEFAULT_VOXEL_SIZE,
        learning_duration_s:  float = DEFAULT_LEARNING_TIME,
        hit_threshold:        float = DEFAULT_HIT_THRESHOLD,
        decay_rate:           float = DEFAULT_DECAY_RATE,
        refresh_rate:         float = DEFAULT_REFRESH_RATE,
        background_threshold: float = DEFAULT_BACKGROUND_THR,
        max_range_m:          float = DEFAULT_MAX_RANGE,
    ):
        self.voxel_size           = voxel_size
        self.learning_duration_s  = learning_duration_s
        self.hit_threshold        = hit_threshold
        self.decay_rate           = decay_rate
        self.refresh_rate         = refresh_rate
        self.background_threshold = background_threshold
        self.max_range_m          = max_range_m

        self._lock = threading.RLock()

        # Voxel state — key is (i, j, k) tuple, value is a float score 0..1
        # During LEARNING: score = hits / frames_observed
        # During ACTIVE:   score continuously updated via refresh/decay
        self._voxels: dict[tuple[int, int, int], float] = defaultdict(float)

        # During LEARNING we count raw hits per voxel
        self._learning_hits: dict[tuple[int, int, int], int] = defaultdict(int)

        # Track which voxels are "frozen" as background after learning
        self._background: set[tuple[int, int, int]] = set()

        self._state          = "LEARNING"
        self._start_time     = time.monotonic()
        self._frames_seen    = 0
        self._decay_events   = 0

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _voxel_key(self, x: float, y: float, z: float) -> tuple[int, int, int]:
        """Quantize a 3D point to its voxel grid key."""
        s = self.voxel_size
        return (
            math.floor(x / s),
            math.floor(y / s),
            math.floor(z / s),
        )

    @staticmethod
    def _point_range(pt) -> float:
        """Euclidean range — works with DetectedPoint or any obj with x/y/z."""
        return math.sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z)

    # ------------------------------------------------------------------
    # Main observation method — call once per frame
    # ------------------------------------------------------------------

    def observe(self, points: list):
        """
        Update the background model with one frame of points.
        Safe to call in any state. Does the right thing automatically.
        """
        with self._lock:
            self._frames_seen += 1

            # Index points by voxel key for this frame
            frame_voxels = set()
            for pt in points:
                # Ignore returns beyond sensor max range — likely spurious
                if self._point_range(pt) > self.max_range_m:
                    continue
                key = self._voxel_key(pt.x, pt.y, pt.z)
                frame_voxels.add(key)

            if self._state == "LEARNING":
                self._observe_learning(frame_voxels)
                self._check_learning_complete()
            else:   # ACTIVE
                self._observe_active(frame_voxels)

    def _observe_learning(self, frame_voxels: set):
        """During learning: count hits per voxel."""
        for key in frame_voxels:
            self._learning_hits[key] += 1

    def _observe_active(self, frame_voxels: set):
        """During active phase: refresh hit voxels, decay all others."""
        # Refresh: voxels we saw this frame
        for key in frame_voxels:
            current = self._voxels.get(key, 0.0)
            self._voxels[key] = min(1.0, current + self.refresh_rate)

        # Decay: every voxel we have on record
        # Use list() so we can mutate during iteration
        to_remove = []
        for key in list(self._voxels.keys()):
            if key in frame_voxels:
                continue
            current = self._voxels[key]
            new = current - self.decay_rate
            if new <= 0.0:
                to_remove.append(key)
            else:
                self._voxels[key] = new

        # Clean up dead voxels and update background set
        for key in to_remove:
            del self._voxels[key]
            if key in self._background:
                self._background.discard(key)
                self._decay_events += 1

        # Re-evaluate background membership based on current scores
        # A voxel becomes background once its score crosses threshold,
        # and stops being background if its score falls below.
        for key, score in self._voxels.items():
            if score >= self.background_threshold:
                self._background.add(key)
            elif key in self._background and score < self.background_threshold * 0.7:
                # Hysteresis — only remove when it falls clearly below
                self._background.discard(key)

    def _check_learning_complete(self):
        """Transition LEARNING → ACTIVE once duration has elapsed."""
        elapsed = time.monotonic() - self._start_time
        if elapsed < self.learning_duration_s:
            return

        # Convert learning hits to initial scores
        frames = max(self._frames_seen, 1)
        for key, hits in self._learning_hits.items():
            score = hits / frames
            if score >= self.hit_threshold:
                self._voxels[key]     = 1.0   # max confidence
                self._background.add(key)
            else:
                # Don't carry forward — keep voxel grid sparse
                pass

        self._learning_hits.clear()
        self._state = "ACTIVE"
        logger.info(
            f"Background learning complete: {len(self._background)} background "
            f"voxels from {frames} frames"
        )

    # ------------------------------------------------------------------
    # Public queries
    # ------------------------------------------------------------------

    def is_novel(self, pt) -> bool:
        """
        Returns True if the point is NOT in the learned background.
        During LEARNING, always returns False (nothing is novel yet).
        """
        with self._lock:
            if self._state == "LEARNING":
                return False
            key = self._voxel_key(pt.x, pt.y, pt.z)
            return key not in self._background

    def filter_novel(self, points: list) -> list:
        """
        Returns the subset of points that are novel (not background).
        During LEARNING, returns empty list — we don't trust detections yet.
        """
        with self._lock:
            if self._state == "LEARNING":
                return []
            return [
                p for p in points
                if self._voxel_key(p.x, p.y, p.z) not in self._background
            ]

    def start_relearn(self):
        """Reset model to LEARNING state, clearing all background."""
        with self._lock:
            logger.info("Background relearn triggered — entering LEARNING state")
            self._voxels.clear()
            self._learning_hits.clear()
            self._background.clear()
            self._state         = "RELEARNING"
            self._start_time    = time.monotonic()
            self._frames_seen   = 0
            self._decay_events  = 0
            # RELEARNING acts like LEARNING; transition happens in observe()
            self._state         = "LEARNING"

    def get_stats(self) -> BackgroundModelStats:
        with self._lock:
            if self._state == "LEARNING":
                elapsed = time.monotonic() - self._start_time
                remaining = max(0.0, self.learning_duration_s - elapsed)
            else:
                remaining = 0.0
            return BackgroundModelStats(
                state             = self._state,
                seconds_remaining = remaining,
                voxel_count       = len(self._voxels),
                background_voxels = len(self._background),
                frames_observed   = self._frames_seen,
                decay_events      = self._decay_events,
            )

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def is_learning(self) -> bool:
        with self._lock:
            return self._state == "LEARNING"


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import logging
    from dataclasses import dataclass
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    @dataclass
    class TestPoint:
        x: float; y: float; z: float

    # Simulate a static wall at y=2.0 with three returns
    wall_points = [
        TestPoint(-0.3, 2.0, 0.0),
        TestPoint( 0.0, 2.0, 0.0),
        TestPoint(+0.3, 2.0, 0.0),
    ]
    # Plus a person at y=1.0 who arrives after learning
    person = TestPoint(0.0, 1.0, 0.0)

    model = BackgroundModel(learning_duration_s=2.0)   # short for test

    print("Phase 1 — Learning (2 seconds at 10Hz = 20 frames)")
    for i in range(20):
        model.observe(wall_points)
        time.sleep(0.1)

    stats = model.get_stats()
    print(f"  state={stats.state}, bg_voxels={stats.background_voxels}")

    print("\nPhase 2 — Active, wall only (should yield no novel returns)")
    for i in range(5):
        model.observe(wall_points)
        novel = model.filter_novel(wall_points)
        print(f"  novel={len(novel)} (expect 0)")

    print("\nPhase 3 — Active, person walks in (should yield 1 novel return)")
    for i in range(5):
        scene = wall_points + [person]
        model.observe(scene)
        novel = model.filter_novel(scene)
        print(f"  novel={len(novel)} at "
              f"{[(p.x, p.y, p.z) for p in novel]} (expect 1 at (0,1,0))")

    print("\nPhase 4 — Trigger relearn")
    model.start_relearn()
    print(f"  state={model.state} (expect LEARNING)")
