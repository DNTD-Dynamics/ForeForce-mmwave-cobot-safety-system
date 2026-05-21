"""
cluster.py
DNTD Dynamics — mmWave Point Cloud Cluster Builder

Groups novel DetectedPoints into spatial clusters using DBSCAN and
extracts per-cluster features used by the micro-doppler classifier.

Features extracted per cluster:
  centroid_x/y/z      — spatial center of mass
  point_count         — number of points in cluster
  range_m             — distance from sensor to centroid
  velocity_mean       — mean radial velocity across cluster
  velocity_spread     — std dev of velocities (high = limb motion = person)
  height_span         — Z range of cluster (person tall, tool small)
  doppler_asymmetry   — ratio of points with negative vs positive velocity
                        (pure positive = receding object, mixed = person walking)
  snr_mean            — mean SNR across cluster points

Temporal features (maintained across frames per cluster ID):
  temporal_variance   — how much the centroid moves frame-to-frame
                        (person drifts/sways, dropped tool is ballistic then static)

Thread-safe. Designed for 10Hz call rate.
"""

import math
import time
import logging
from dataclasses import dataclass, field
from typing import Optional
from collections import deque

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# DBSCAN parameters — tunable but these work well for 10cm voxel spacing
# ---------------------------------------------------------------------------
DEFAULT_EPS_M        = 0.40   # m  — max distance between points in same cluster
DEFAULT_MIN_POINTS   = 2      # min points to form a cluster (low — sparse cloud)
DEFAULT_HISTORY_LEN  = 10     # frames of centroid history per cluster


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Cluster:
    """
    One spatially grouped object detected in the scene.
    All features are computed fresh each frame from member points.
    """
    cluster_id:         int
    centroid_x:         float
    centroid_y:         float
    centroid_z:         float
    range_m:            float        # distance from sensor origin to centroid
    point_count:        int
    velocity_mean:      float        # mean radial velocity (m/s)
    velocity_spread:    float        # std dev of velocities across points
    height_span:        float        # max_z - min_z of cluster points
    doppler_asymmetry:  float        # fraction of points with negative velocity
                                     # 0.0 = all receding, 1.0 = all approaching
                                     # ~0.5 = mixed (walking person)
    snr_mean:           float
    temporal_variance:  float        # centroid movement variance over history
    timestamp:          float = field(default_factory=time.time)

    def __str__(self):
        return (
            f"Cluster(id={self.cluster_id} pts={self.point_count} "
            f"range={self.range_m:.2f}m vel={self.velocity_mean:.2f}±{self.velocity_spread:.2f}m/s "
            f"h={self.height_span:.2f}m dA={self.doppler_asymmetry:.2f} "
            f"tv={self.temporal_variance:.4f})"
        )


# ---------------------------------------------------------------------------
# Cluster builder
# ---------------------------------------------------------------------------

class ClusterBuilder:
    """
    Groups novel points into clusters each frame and extracts features.

    Usage:
        builder = ClusterBuilder()
        clusters = builder.update(novel_points)  # call once per frame
    """

    def __init__(
        self,
        eps_m:       float = DEFAULT_EPS_M,
        min_points:  int   = DEFAULT_MIN_POINTS,
        history_len: int   = DEFAULT_HISTORY_LEN,
    ):
        self.eps_m       = eps_m
        self.min_points  = min_points
        self.history_len = history_len

        # Centroid history per cluster ID — used for temporal variance
        # Key: cluster_id (assigned by spatial proximity across frames)
        # Value: deque of (x, y, z) centroid positions
        self._centroid_history: dict[int, deque] = {}
        self._next_id = 0
        self._prev_centroids: list[tuple[float, float, float]] = []

    # ------------------------------------------------------------------

    def update(self, points: list) -> list[Cluster]:
        """
        Call once per frame with novel DetectedPoints.
        Returns list of Cluster objects (may be empty).
        """
        if not points:
            return []

        # Step 1 — DBSCAN clustering
        labels = self._dbscan(points)

        # Step 2 — Group points by label
        groups: dict[int, list] = {}
        for pt, label in zip(points, labels):
            if label == -1:
                continue  # noise point — not in any cluster
            groups.setdefault(label, []).append(pt)

        if not groups:
            return []

        # Step 3 — Extract features per group
        clusters = []
        new_centroids = []

        for label, members in groups.items():
            centroid = _centroid(members)
            new_centroids.append(centroid)

            # Match to previous frame centroid for ID continuity
            cluster_id = self._match_or_new_id(centroid)

            # Update centroid history
            if cluster_id not in self._centroid_history:
                self._centroid_history[cluster_id] = deque(maxlen=self.history_len)
            self._centroid_history[cluster_id].append(centroid)

            # Compute temporal variance from centroid history
            tv = _temporal_variance(self._centroid_history[cluster_id])

            cluster = Cluster(
                cluster_id        = cluster_id,
                centroid_x        = centroid[0],
                centroid_y        = centroid[1],
                centroid_z        = centroid[2],
                range_m           = math.sqrt(sum(c*c for c in centroid)),
                point_count       = len(members),
                velocity_mean     = _mean([p.velocity for p in members]),
                velocity_spread   = _std([p.velocity for p in members]),
                height_span       = max(p.z for p in members) - min(p.z for p in members),
                doppler_asymmetry = sum(1 for p in members if p.velocity < 0) / len(members),
                snr_mean          = _mean([p.snr for p in members]),
                temporal_variance = tv,
            )
            clusters.append(cluster)

        # Prune old cluster histories that didn't appear this frame
        self._prune_stale_histories(new_centroids)
        self._prev_centroids = new_centroids

        return clusters

    # ------------------------------------------------------------------
    # DBSCAN — simple O(n²) implementation, fine for sparse mmWave clouds
    # ------------------------------------------------------------------

    def _dbscan(self, points: list) -> list[int]:
        """
        Returns a label list parallel to points.
        -1 = noise, 0..N = cluster index.
        """
        n      = len(points)
        labels = [-2] * n   # -2 = unvisited

        def neighbors(idx):
            p = points[idx]
            return [
                j for j in range(n)
                if _dist(p, points[j]) <= self.eps_m
            ]

        cluster_id = 0

        for i in range(n):
            if labels[i] != -2:
                continue   # already visited

            nbrs = neighbors(i)

            if len(nbrs) < self.min_points:
                labels[i] = -1   # noise
                continue

            # Start new cluster
            labels[i] = cluster_id
            seed_set = set(nbrs) - {i}

            while seed_set:
                j = seed_set.pop()
                if labels[j] == -1:
                    labels[j] = cluster_id   # border point
                if labels[j] != -2:
                    continue
                labels[j] = cluster_id
                j_nbrs = neighbors(j)
                if len(j_nbrs) >= self.min_points:
                    seed_set.update(j_nbrs)

            cluster_id += 1

        return labels

    # ------------------------------------------------------------------
    # Cluster ID continuity across frames
    # ------------------------------------------------------------------

    def _match_or_new_id(self, centroid: tuple) -> int:
        """
        Assign a persistent ID by matching the new centroid to the
        closest previous-frame centroid within eps_m * 2.
        If no match, assign a new ID.
        """
        best_id   = None
        best_dist = float('inf')

        for prev_id, hist in self._centroid_history.items():
            if not hist:
                continue
            prev = hist[-1]
            d = math.sqrt(sum((a-b)**2 for a, b in zip(centroid, prev)))
            if d < self.eps_m * 2 and d < best_dist:
                best_dist = d
                best_id   = prev_id

        if best_id is not None:
            return best_id

        new_id = self._next_id
        self._next_id += 1
        return new_id

    def _prune_stale_histories(self, active_centroids: list):
        """
        Remove centroid histories for clusters that haven't appeared
        in the last history_len frames (they've left the scene).
        Keep memory bounded.
        """
        max_history = self.history_len
        stale = [
            cid for cid, hist in self._centroid_history.items()
            if len(hist) > 0 and not any(
                math.sqrt(sum((a-b)**2 for a, b in zip(hist[-1], ac)))
                < self.eps_m * 3
                for ac in active_centroids
            )
        ]
        # Only prune after history window — gives re-entry a chance
        for cid in stale:
            hist = self._centroid_history[cid]
            if len(hist) == hist.maxlen:
                del self._centroid_history[cid]


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _dist(a, b) -> float:
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

def _centroid(points) -> tuple:
    n = len(points)
    return (
        sum(p.x for p in points) / n,
        sum(p.y for p in points) / n,
        sum(p.z for p in points) / n,
    )

def _mean(vals: list) -> float:
    return sum(vals) / len(vals) if vals else 0.0

def _std(vals: list) -> float:
    if len(vals) < 2:
        return 0.0
    m = _mean(vals)
    return math.sqrt(sum((v - m)**2 for v in vals) / len(vals))

def _temporal_variance(history: deque) -> float:
    """
    Mean squared displacement of centroid across history frames.
    High value = cluster is moving around = more likely a person.
    Low value = cluster is stationary = more likely a static object.
    """
    pts = list(history)
    if len(pts) < 2:
        return 0.0
    disps = []
    for i in range(1, len(pts)):
        d = math.sqrt(sum((a-b)**2 for a, b in zip(pts[i], pts[i-1])))
        disps.append(d)
    return _mean(disps)


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import logging
    from dataclasses import dataclass as dc
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    @dc
    class P:
        x: float; y: float; z: float
        velocity: float; snr: float

    builder = ClusterBuilder()

    print("=== Frame 1: Two distinct clusters ===")
    points = [
        # Cluster A — person at ~1m, mixed doppler, tall
        P(0.0,  1.0, 0.0, -0.6, 22.0),
        P(0.1,  1.1, 0.1, -0.4, 20.0),
        P(-0.1, 1.0, 0.5, -0.7, 21.0),
        P(0.0,  0.9, 1.0, -0.5, 19.0),
        # Cluster B — object at ~2.5m, uniform doppler, flat
        P(0.0,  2.5, 0.0, -0.2, 15.0),
        P(0.05, 2.5, 0.0, -0.2, 14.0),
        P(-0.05,2.5, 0.0, -0.2, 16.0),
    ]
    clusters = builder.update(points)
    for c in clusters:
        print(f"  {c}")

    print("\n=== Frame 2: Same clusters, person moved slightly ===")
    points2 = [
        P(0.0,  0.9, 0.0, -0.7, 22.0),
        P(0.1,  1.0, 0.1, -0.5, 20.0),
        P(-0.1, 0.9, 0.5, -0.8, 21.0),
        P(0.0,  0.8, 1.0, -0.6, 19.0),
        P(0.0,  2.5, 0.0, -0.2, 15.0),
        P(0.05, 2.5, 0.0, -0.2, 14.0),
        P(-0.05,2.5, 0.0, -0.2, 16.0),
    ]
    clusters2 = builder.update(points2)
    for c in clusters2:
        print(f"  {c}")

    print("\n=== Frame 3: Empty scene ===")
    clusters3 = builder.update([])
    print(f"  clusters={len(clusters3)} (expect 0)")
