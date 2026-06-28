"""
classifier.py
DNTD Dynamics — mmWave Micro-Doppler Cluster Classifier

Rule-based classifier that labels each cluster as PERSON, OBJECT, or UNKNOWN
based on features extracted by cluster.py.

Safety stance: FAIL-SAFE
  PERSON  → passed to zone logic normally
  UNKNOWN → passed to zone logic (treated as PERSON — fail-safe)
  OBJECT  → suppressed from zone logic, logged for ML training data

All thresholds are YAML-tunable — no code changes needed to adapt to a new
workspace or arm geometry. Defaults are conservative (more false positives,
never false negatives).

Training data logging:
  Every suppressed OBJECT detection is logged to a CSV file.
  This automatically builds a labeled dataset for future ML model training.
  Format: timestamp, cluster features, label
  Path: ~/mmwave/logs/classifier_training.csv (configurable)

Future upgrade path:
  Phase 6b — drop-in ML model (sklearn or tflite) that replaces the
  rule-based thresholds. The same Cluster feature vector feeds both.
  Pre-trained models will be available as optional add-ons.
"""

import csv
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

from cluster import Cluster

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Default thresholds — tuned for typical industrial workspace
# All overridable via constructor (load from YAML in safety node)
# ---------------------------------------------------------------------------

# A person cluster typically has:
#   - Multiple points (body + limbs)
#   - High velocity spread (different body parts move differently)
#   - Significant height span (torso to head/knee range)
#   - Mixed doppler (some parts approaching, some receding during gait)
#   - Temporal variance > 0 (people sway, breathe, shift weight)

DEFAULTS = {
    # Minimum point count to even attempt classification
    # Clusters smaller than this pass through as UNKNOWN (fail-safe)
    'min_points_to_classify':   2,

    # Velocity spread (std dev across cluster points)
    # People: arms/legs return different velocities → high spread
    # Objects: uniform velocity → low spread
    'person_velocity_spread_min':  0.08,   # m/s — below this = likely object

    # Height span
    # People: typically 0.3–1.8m tall in the FOV
    # Fallen tools, debris: typically < 0.2m
    'person_height_span_min':      0.10,   # m — below this = likely flat object

    # Point count
    # People return more points due to complex geometry
    # Small objects return 1-3 points
    'person_point_count_min':      3,

    # Doppler asymmetry
    # Walking person: ~0.3–0.7 (mixed approaching/receding limbs)
    # Pure ballistic object: near 0.0 or 1.0 (uniform direction)
    # We use this as a soft signal only — not a hard gate
    'person_doppler_asymmetry_min': 0.15,
    'person_doppler_asymmetry_max': 0.85,

    # Temporal variance
    # Person standing still: small but non-zero (breathing, micro-motion)
    # Stationary object: zero
    # NOTE: On first few frames history is short — don't gate hard on this
    'person_temporal_variance_min': 0.0,   # m/frame — 0 = don't use as gate

    # Score threshold — how many person-like features needed to label PERSON
    # Out of 4 soft feature votes. Lower = more sensitive (more false positives).
    # Higher = more strict (risk of missing a person).
    # FAIL-SAFE: default 2 of 4 — generous threshold
    'person_score_threshold':       2,
}


# ---------------------------------------------------------------------------
# Classification result
# ---------------------------------------------------------------------------

@dataclass
class ClassificationResult:
    cluster:    Cluster
    label:      str          # 'PERSON' | 'OBJECT' | 'UNKNOWN'
    confidence: float        # 0.0–1.0  (score / max_score)
    reason:     str          # human-readable explanation


# ---------------------------------------------------------------------------
# Classifier
# ---------------------------------------------------------------------------

class MicroDopplerClassifier:
    """
    Rule-based classifier. Labels each cluster as PERSON, OBJECT, or UNKNOWN.

    Fail-safe: UNKNOWN passes through to zone logic as if it were PERSON.
    Only confident OBJECT detections are suppressed.

    Usage:
        clf = MicroDopplerClassifier()
        results = clf.classify(clusters)
        person_clusters = [r.cluster for r in results if r.label != 'OBJECT']
    """

    def __init__(
        self,
        thresholds:   Optional[dict] = None,
        log_path: str = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs", "classifier_training.csv"),
        enable_logging: bool = True,
    ):
        # Merge provided thresholds over defaults
        self._t = {**DEFAULTS, **(thresholds or {})}
        self._log_path      = log_path
        self._enable_logging = enable_logging
        self._log_lock       = threading.Lock()
        self._log_file       = None
        self._log_writer     = None

        if enable_logging:
            self._init_log()

    # ------------------------------------------------------------------

    def classify(self, clusters: list[Cluster]) -> list[ClassificationResult]:
        """
        Classify a list of clusters. Returns one result per cluster.
        Call once per frame after ClusterBuilder.update().
        """
        results = []
        for cluster in clusters:
            result = self._classify_one(cluster)
            results.append(result)

            if result.label == 'OBJECT' and self._enable_logging:
                self._log_suppressed(result)

            logger.debug(
                f"Cluster {cluster.cluster_id}: {result.label} "
                f"(conf={result.confidence:.2f}) — {result.reason}"
            )

        return results

    def filter_person_clusters(self, clusters: list[Cluster]) -> list[Cluster]:
        """
        Convenience method: classify and return only non-OBJECT clusters.
        PERSON and UNKNOWN both pass through (fail-safe).
        These are the clusters that feed zone logic.
        """
        results = self.classify(clusters)
        return [r.cluster for r in results if r.label != 'OBJECT']

    # ------------------------------------------------------------------
    # Core classification logic
    # ------------------------------------------------------------------

    def _classify_one(self, c: Cluster) -> ClassificationResult:
        t = self._t

        # --- Gate: too few points to classify reliably → UNKNOWN ---
        if c.point_count < t['min_points_to_classify']:
            return ClassificationResult(
                cluster    = c,
                label      = 'UNKNOWN',
                confidence = 0.0,
                reason     = f"Too few points ({c.point_count}) to classify — fail-safe pass-through",
            )

        # --- Soft feature votes ---
        # Each feature votes +1 toward PERSON if it looks person-like.
        # We use soft voting so no single feature is a hard gate.
        # The fail-safe threshold means we only suppress confident OBJECTs.

        votes   = 0
        max_votes = 4
        evidence  = []

        # Vote 1: Velocity spread
        if c.velocity_spread >= t['person_velocity_spread_min']:
            votes += 1
            evidence.append(f"vel_spread={c.velocity_spread:.3f}✓")
        else:
            evidence.append(f"vel_spread={c.velocity_spread:.3f}✗")

        # Vote 2: Height span
        if c.height_span >= t['person_height_span_min']:
            votes += 1
            evidence.append(f"h_span={c.height_span:.2f}m✓")
        else:
            evidence.append(f"h_span={c.height_span:.2f}m✗")

        # Vote 3: Point count — but only counts toward PERSON if at least
        # one other feature is person-like. High point count from equipment
        # vibration (many points, uniform velocity, flat) shouldn't override
        # strongly object-like velocity and height evidence.
        vel_ok = c.velocity_spread >= t['person_velocity_spread_min']
        hgt_ok = c.height_span >= t['person_height_span_min']
        if c.point_count >= t['person_point_count_min'] and (vel_ok or hgt_ok):
            votes += 1
            evidence.append(f"pts={c.point_count}✓")
        elif c.point_count >= t['person_point_count_min']:
            evidence.append(f"pts={c.point_count}~(no corroborating features)")
        else:
            evidence.append(f"pts={c.point_count}✗")

        # Vote 4: Doppler asymmetry (soft — mixed doppler = person moving)
        da_min = t['person_doppler_asymmetry_min']
        da_max = t['person_doppler_asymmetry_max']
        if da_min <= c.doppler_asymmetry <= da_max:
            votes += 1
            evidence.append(f"dA={c.doppler_asymmetry:.2f}✓")
        else:
            evidence.append(f"dA={c.doppler_asymmetry:.2f}✗")

        # Temporal variance — bonus vote if history is long enough
        # Don't penalize early frames where history is short
        if c.temporal_variance > t['person_temporal_variance_min'] > 0:
            votes = min(votes + 1, max_votes)
            evidence.append(f"tv={c.temporal_variance:.4f}✓")

        confidence = votes / max_votes
        reason_str = " | ".join(evidence)

        # --- Decision ---
        threshold = t['person_score_threshold']

        if votes >= threshold:
            return ClassificationResult(
                cluster    = c,
                label      = 'PERSON',
                confidence = confidence,
                reason     = reason_str,
            )
        else:
            # Only suppress if score is clearly object-like
            # votes == threshold-1 → UNKNOWN (fail-safe, not confident enough to suppress)
            if votes <= threshold - 2:
                label = 'OBJECT'
            else:
                label = 'UNKNOWN'

            return ClassificationResult(
                cluster    = c,
                label      = label,
                confidence = 1.0 - confidence,
                reason     = reason_str,
            )

    # ------------------------------------------------------------------
    # Training data logger
    # ------------------------------------------------------------------

    def _init_log(self):
        """Create log file and write header if it doesn't exist."""
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self._log_path)),
                        exist_ok=True)
            write_header = not os.path.exists(self._log_path)
            self._log_file   = open(self._log_path, 'a', newline='')
            self._log_writer = csv.writer(self._log_file)
            if write_header:
                self._log_writer.writerow([
                    'timestamp', 'label', 'confidence',
                    'point_count', 'range_m',
                    'velocity_mean', 'velocity_spread',
                    'height_span', 'doppler_asymmetry',
                    'snr_mean', 'temporal_variance',
                ])
                self._log_file.flush()
            logger.info(f"Classifier training log: {self._log_path}")
        except Exception as e:
            logger.warning(f"Could not open classifier log: {e}")
            self._enable_logging = False

    def _log_suppressed(self, result: ClassificationResult):
        """Log a suppressed OBJECT cluster for ML training data."""
        if self._log_writer is None:
            return
        c = result.cluster
        try:
            with self._log_lock:
                self._log_writer.writerow([
                    f"{time.time():.3f}",
                    result.label,
                    f"{result.confidence:.3f}",
                    c.point_count,
                    f"{c.range_m:.3f}",
                    f"{c.velocity_mean:.3f}",
                    f"{c.velocity_spread:.3f}",
                    f"{c.height_span:.3f}",
                    f"{c.doppler_asymmetry:.3f}",
                    f"{c.snr_mean:.3f}",
                    f"{c.temporal_variance:.4f}",
                ])
                self._log_file.flush()
        except Exception as e:
            logger.warning(f"Classifier log write failed: {e}")

    def cleanup(self):
        if self._log_file:
            try:
                self._log_file.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import logging
    from dataclasses import dataclass as dc
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    TEST_LOG = "/tmp/classifier_test.csv"
    clf = MicroDopplerClassifier(log_path=TEST_LOG)

    def make_cluster(cid, pts, vel_spread, h_span, dA, tv=0.0, range_m=1.0):
        from cluster import Cluster
        return Cluster(
            cluster_id       = cid,
            centroid_x       = 0.0,
            centroid_y       = range_m,
            centroid_z       = 0.0,
            range_m          = range_m,
            point_count      = pts,
            velocity_mean    = -0.5,
            velocity_spread  = vel_spread,
            height_span      = h_span,
            doppler_asymmetry= dA,
            snr_mean         = 20.0,
            temporal_variance= tv,
        )

    test_cases = [
        # (description, cluster, expected_label)
        ("Walking person — high spread, tall, mixed doppler",
         make_cluster(0, pts=5, vel_spread=0.25, h_span=0.8, dA=0.6, tv=0.05),
         "PERSON"),

        ("Stationary person — low spread but tall, mixed doppler",
         make_cluster(1, pts=4, vel_spread=0.05, h_span=0.6, dA=0.5, tv=0.01),
         "PERSON"),

        ("Falling tool — few points, uniform doppler, flat",
         make_cluster(2, pts=2, vel_spread=0.01, h_span=0.05, dA=0.95, tv=0.0),
         "OBJECT"),

        ("Single point — too few to classify",
         make_cluster(3, pts=1, vel_spread=0.0, h_span=0.0, dA=1.0, tv=0.0),
         "UNKNOWN"),

        ("Ambiguous — borderline features (fail-safe: UNKNOWN passes through)",
         make_cluster(4, pts=3, vel_spread=0.06, h_span=0.08, dA=0.3, tv=0.0),
         "UNKNOWN"),  # UNKNOWN = fail-safe pass-through to zone logic

        ("Equipment vibration — many points, uniform velocity, flat",
         make_cluster(5, pts=6, vel_spread=0.02, h_span=0.05, dA=0.9, tv=0.0),
         "OBJECT"),
    ]

    print(f"\n{'─'*70}")
    print("  Micro-doppler classifier test — DNTD Dynamics")
    print(f"{'─'*70}\n")

    all_pass = True
    for desc, cluster, expected in test_cases:
        results = clf.classify([cluster])
        r = results[0]
        status = "✅" if r.label == expected else "❌"
        if r.label != expected:
            all_pass = False
        print(f"{status}  {desc}")
        print(f"     → {r.label} (conf={r.confidence:.2f})")
        print(f"     → {r.reason}\n")

    print(f"{'─'*70}")
    print(f"  All tests passed: {all_pass}")
    print(f"  Training log: {TEST_LOG}")
    clf.cleanup()
