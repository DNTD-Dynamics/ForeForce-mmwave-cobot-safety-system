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

Persistence:
  After learning completes, the background map is automatically saved to disk.
  On startup, if a valid saved map exists it is loaded and the 15s learning
  phase is skipped entirely — the sensor goes straight to ACTIVE.
  Calling start_relearn() deletes the saved map and re-enters LEARNING.
  Save path defaults to ~/mmwave/configs/background_map.npz (configurable).

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
import os
import numpy as np
from collections import defaultdict
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

# Bump this when the save format changes — old maps will be rejected cleanly
_SAVE_FORMAT_VERSION = 1


@dataclass
class BackgroundModelStats:
    """Snapshot of model state for diagnostics."""
    state:               str       # LEARNING | ACTIVE | RELEARNING
    seconds_remaining:   float     # in learning phase, 0 once ACTIVE
    voxel_count:         int       # total voxels with any observations
    background_voxels:   int       # voxels classified as background
    frames_observed:     int       # total frames seen since model start
    decay_events:        int       # how many voxels have decayed below threshold
    loaded_from_disk:    bool      # True if this session skipped learning


class BackgroundModel:
    """
    Voxel-grid background learner with automatic persistence.

    On construction:
      - If a valid saved map exists at map_path, loads it and enters ACTIVE
        immediately — no learning phase required.
      - Otherwise enters LEARNING as normal.

    After learning completes:
      - Automatically saves the background map to map_path.

    On start_relearn():
      - Deletes the saved map and re-enters LEARNING.

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
    DEFAULT_MAP_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs", "background_map.npz")

    def __init__(
        self,
        voxel_size:           float = DEFAULT_VOXEL_SIZE,
        learning_duration_s:  float = DEFAULT_LEARNING_TIME,
        hit_threshold:        float = DEFAULT_HIT_THRESHOLD,
        decay_rate:           float = DEFAULT_DECAY_RATE,
        refresh_rate:         float = DEFAULT_REFRESH_RATE,
        background_threshold: float = DEFAULT_BACKGROUND_THR,
        max_range_m:          float = DEFAULT_MAX_RANGE,
        map_path:             str   = DEFAULT_MAP_PATH,
    ):
        self.voxel_size           = voxel_size
        self.learning_duration_s  = learning_duration_s
        self.hit_threshold        = hit_threshold
        self.decay_rate           = decay_rate
        self.refresh_rate         = refresh_rate
        self.background_threshold = background_threshold
        self.max_range_m          = max_range_m
        self.map_path             = map_path

        self._lock = threading.RLock()

        # Voxel state — key is (i, j, k) tuple, value is a float score 0..1
        # During LEARNING: score = hits / frames_observed
        # During ACTIVE:   score continuously updated via refresh/decay
        self._voxels: dict[tuple[int, int, int], float] = defaultdict(float)

        # During LEARNING we count raw hits per voxel
        self._learning_hits: dict[tuple[int, int, int], int] = defaultdict(int)

        # Track which voxels are "frozen" as background after learning
        self._background: set[tuple[int, int, int]] = set()

        self._state           = "LEARNING"
        self._start_time      = time.monotonic()
        self._frames_seen     = 0
        self._decay_events    = 0
        self._loaded_from_disk = False

        # Try to load a previously saved map — skips learning if successful
        self._try_load_map()

    # ------------------------------------------------------------------
    # Persistence — save / load
    # ------------------------------------------------------------------

    def _try_load_map(self):
        """
        Attempt to load a saved background map from disk.
        If successful, transition directly to ACTIVE — no learning needed.
        Silently ignores missing or incompatible files.
        """
        if not os.path.exists(self.map_path):
            logger.info(
                f"No saved background map at {self.map_path} — "
                "entering learning phase"
            )
            return

        try:
            data = np.load(self.map_path, allow_pickle=False)

            # Version check — reject maps saved by incompatible code
            version = int(data['version'])
            if version != _SAVE_FORMAT_VERSION:
                logger.warning(
                    f"Saved map version {version} != {_SAVE_FORMAT_VERSION} — "
                    "ignoring, will relearn"
                )
                return

            # Voxel size must match — a different resolution means the keys
            # are in different coordinate systems
            saved_voxel_size = float(data['voxel_size'])
            if not math.isclose(saved_voxel_size, self.voxel_size, rel_tol=1e-4):
                logger.warning(
                    f"Saved map voxel size {saved_voxel_size}m != "
                    f"{self.voxel_size}m — ignoring, will relearn"
                )
                return

            # Load voxel keys and scores
            keys_array   = data['keys']    # shape (N, 3), dtype int32
            scores_array = data['scores']  # shape (N,),   dtype float32
            bg_array     = data['background']  # shape (M, 3), dtype int32

            for k, s in zip(keys_array, scores_array):
                self._voxels[tuple(k)] = float(s)

            for k in bg_array:
                self._background.add(tuple(k))

            self._state            = "ACTIVE"
            self._loaded_from_disk = True

            logger.info(
                f"Loaded background map from {self.map_path}: "
                f"{len(self._background)} background voxels — "
                "skipping learning phase"
            )

        except Exception as e:
            logger.warning(
                f"Failed to load background map ({e}) — entering learning phase"
            )
            # Reset to clean state in case partial load occurred
            self._voxels.clear()
            self._background.clear()
            self._state = "LEARNING"

    def _save_map(self):
        """
        Save the current background map to disk.
        Called automatically when learning completes.
        Runs in a background thread so it never blocks the sensor pipeline.
        """
        threading.Thread(
            target=self._save_map_worker,
            daemon=True,
            name='bg_map_save',
        ).start()

    def _save_map_worker(self):
        """Worker — runs off the hot path."""
        try:
            # Snapshot under lock, then release before doing I/O
            with self._lock:
                keys_list   = list(self._voxels.keys())
                scores_list = [self._voxels[k] for k in keys_list]
                bg_list     = list(self._background)

            keys_array   = np.array(keys_list,   dtype=np.int32)
            scores_array = np.array(scores_list, dtype=np.float32)
            bg_array     = np.array(bg_list,     dtype=np.int32) \
                           if bg_list else np.zeros((0, 3), dtype=np.int32)

            # Atomic write — save to .tmp then rename so a crash mid-write
            # never leaves a corrupt map on disk
            abs_path = os.path.abspath(self.map_path)
            # np.savez_compressed appends .npz automatically — strip it from
            # the tmp stem so the rename lands on the correct final path
            tmp_stem = (abs_path[:-4] if abs_path.endswith('.npz') else abs_path) + ".tmp"
            os.makedirs(os.path.dirname(abs_path), exist_ok=True)
            np.savez_compressed(
                tmp_stem,   # numpy writes tmp_stem.npz
                version    = np.array(_SAVE_FORMAT_VERSION, dtype=np.int32),
                voxel_size = np.array(self.voxel_size,       dtype=np.float32),
                keys       = keys_array,
                scores     = scores_array,
                background = bg_array,
            )
            os.replace(tmp_stem + ".npz", abs_path)

            logger.info(
                f"Background map saved to {self.map_path} "
                f"({len(bg_list)} background voxels)"
            )

        except Exception as e:
            logger.error(f"Failed to save background map: {e}")

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
        """During active phase: refresh hit voxels, decay all others.

        Novelty-aware refresh gate: voxels currently flagged as novel
        (not in background) are never refreshed into the background score.
        This prevents a person standing still from gradually being absorbed
        into the background map and disappearing from detection.
        """
        # Refresh: only voxels that are already background — novel voxels
        # are intentionally excluded so people never get masked out
        for key in frame_voxels:
            if key in self._background:
                current = self._voxels.get(key, 0.0)
                self._voxels[key] = min(1.0, current + self.refresh_rate)
            # Novel voxels: do nothing — they stay novel

        # Decay: every voxel we have on record
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

        self._learning_hits.clear()
        self._state = "ACTIVE"

        logger.info(
            f"Background learning complete: {len(self._background)} background "
            f"voxels from {frames} frames — saving map"
        )

        # Auto-save — runs in background thread, never blocks sensor pipeline
        self._save_map()

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
        """
        Reset model to LEARNING state, clearing all background.
        Also deletes the saved map so the next boot does a fresh learn.
        """
        with self._lock:
            logger.info("Background relearn triggered — entering LEARNING state")
            self._voxels.clear()
            self._learning_hits.clear()
            self._background.clear()
            self._state            = "LEARNING"
            self._start_time       = time.monotonic()
            self._frames_seen      = 0
            self._decay_events     = 0
            self._loaded_from_disk = False

        # Delete saved map outside the lock — I/O doesn't need it
        if os.path.exists(self.map_path):
            try:
                os.remove(self.map_path)
                logger.info(f"Deleted saved background map: {self.map_path}")
            except Exception as e:
                logger.warning(f"Could not delete saved map: {e}")

    def get_stats(self) -> BackgroundModelStats:
        with self._lock:
            if self._state == "LEARNING":
                elapsed = time.monotonic() - self._start_time
                remaining = max(0.0, self.learning_duration_s - elapsed)
            else:
                remaining = 0.0
            return BackgroundModelStats(
                state              = self._state,
                seconds_remaining  = remaining,
                voxel_count        = len(self._voxels),
                background_voxels  = len(self._background),
                frames_observed    = self._frames_seen,
                decay_events       = self._decay_events,
                loaded_from_disk   = self._loaded_from_disk,
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

    # Use a temp path so the test doesn't touch your real map
    TEST_MAP = "/tmp/bg_test_map.npz"

    # Clean up any leftover from a previous test run
    if os.path.exists(TEST_MAP):
        os.remove(TEST_MAP)

    wall_points = [
        TestPoint(-0.3, 2.0, 0.0),
        TestPoint( 0.0, 2.0, 0.0),
        TestPoint(+0.3, 2.0, 0.0),
    ]
    person = TestPoint(0.0, 1.0, 0.0)

    print("=== Run 1: Fresh learn ===")
    model = BackgroundModel(learning_duration_s=2.0, map_path=TEST_MAP)
    print(f"  state={model.state} (expect LEARNING)")

    print("Phase 1 — Learning (2 seconds at 10Hz = 20 frames)")
    for i in range(20):
        model.observe(wall_points)
        time.sleep(0.1)

    stats = model.get_stats()
    print(f"  state={stats.state}, bg_voxels={stats.background_voxels}, "
          f"loaded_from_disk={stats.loaded_from_disk}")

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

    print("\nPhase 3b — Person stands still for 200 frames (should stay novel, never absorbed)")
    for i in range(200):
        scene = wall_points + [person]
        model.observe(scene)
    novel = model.filter_novel([person])
    print(f"  novel after 200 frames standing still={len(novel)} (expect 1 — not absorbed)")

    # Give the background save thread time to finish
    time.sleep(1.5)
    print(f"\n  Map saved: {os.path.exists(TEST_MAP)}")

    print("\n=== Run 2: Load from disk — should skip learning ===")
    model2 = BackgroundModel(learning_duration_s=2.0, map_path=TEST_MAP)
    stats2 = model2.get_stats()
    print(f"  state={stats2.state} (expect ACTIVE)")
    print(f"  bg_voxels={stats2.background_voxels} (expect same as run 1)")
    print(f"  loaded_from_disk={stats2.loaded_from_disk} (expect True)")

    print("\nPhase — person detection immediately, no relearn needed")
    for i in range(3):
        scene = wall_points + [person]
        novel = model2.filter_novel(scene)
        print(f"  novel={len(novel)} (expect 1)")

    print("\n=== Run 3: Trigger relearn — map should be deleted ===")
    model2.start_relearn()
    print(f"  state={model2.state} (expect LEARNING)")
    print(f"  map deleted: {not os.path.exists(TEST_MAP)} (expect True)")
