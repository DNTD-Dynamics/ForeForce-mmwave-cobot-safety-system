"""
swept_volume.py
DNTD Dynamics — Arm Swept-Volume Workspace Clipper

Defines the reachable workspace of a robot arm from the sensor mount point
and filters detections to only those inside the dangerous volume.

Design intent (3× IWR6843AOP on forearm link of 6-DOF arm):
  Sensors are mounted on the forearm link (between joint 3 and joint 4).
  Everything distal — joints 4, 5, 6, end effector — sweeps through the
  workspace in front of the sensors. The swept-volume boundary is defined
  by the reach of those distal joints from the sensor mount position in
  world space.

  A detection OUTSIDE the reachable envelope cannot be reached by the arm
  in its current configuration — suppress it.

  A detection INSIDE the self-exclusion zone is the arm's own body —
  suppress it (background model handles static returns, but mid-motion
  the arm can briefly appear as novel).

Geometry:
  - Mount point: world-frame position of the sensor mount link at current q
  - Max reach sphere: radius = sum of distal link lengths from mount point
  - Self-exclusion sphere: radius = arm_body_radius_m around mount point
  - Valid detection zone: between self-exclusion and max reach

Fail-safe behavior:
  - swept_volume_enabled: false → all points pass through unchanged
  - If chain has placeholder geometry → pass all points through
  - Any point that cannot be evaluated → passes through (never silently drop)

Thread-safe. Designed for 10Hz call rate.
"""

import math
import logging
import numpy as np
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Default parameters
# ---------------------------------------------------------------------------

DEFAULT_MAX_REACH_M       = 0.0    # 0 = auto-compute from chain link lengths
DEFAULT_SELF_RADIUS_M     = 0.15   # m — arm body exclusion radius
DEFAULT_REACH_MARGIN_M    = 0.20   # m — safety margin added to max reach
                                   # catches detections just outside nominal reach
DEFAULT_MOUNT_JOINT_IDX   = 3      # 0-indexed joint the sensors are mounted at
                                   # joint 3 = 4th joint on a 6-DOF arm (0-based)


# ---------------------------------------------------------------------------
# Workspace geometry snapshot
# ---------------------------------------------------------------------------

@dataclass
class WorkspaceSnapshot:
    """
    Computed workspace geometry for the current joint configuration.
    Recomputed each frame.
    """
    mount_position:  np.ndarray   # (3,) world-frame position of sensor mount link
    max_reach_m:     float        # outer boundary radius from mount position
    self_radius_m:   float        # inner exclusion radius (arm body)
    reach_margin_m:  float        # margin added to max_reach
    enabled:         bool         # False = bypass, all points pass through

    @property
    def outer_radius(self) -> float:
        return self.max_reach_m + self.reach_margin_m

    def contains(self, x: float, y: float, z: float) -> bool:
        """
        Returns True if the point falls within the valid detection zone:
          self_radius_m < distance_from_mount < max_reach_m + reach_margin_m

        Points inside self_radius are the arm's own body — suppressed.
        Points outside outer_radius are beyond reach — suppressed.
        Points in between are in the dangerous workspace — pass through.
        """
        dx = x - self.mount_position[0]
        dy = y - self.mount_position[1]
        dz = z - self.mount_position[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        return self.self_radius_m < dist <= self.outer_radius


# ---------------------------------------------------------------------------
# Swept volume clipper
# ---------------------------------------------------------------------------

class SweptVolumeClipper:
    """
    Computes the arm's reachable workspace from the sensor mount point
    and filters detections to only those in the dangerous zone.

    Usage:
        clipper = SweptVolumeClipper(chain, mount_joint_idx=3)
        workspace = clipper.update(q)           # call each frame
        safe_points = clipper.filter(points, q) # convenience method

    The clipper needs the KinematicChain to walk joints and find the
    mount point world-frame position and distal link lengths.
    """

    def __init__(
        self,
        chain,                              # KinematicChain instance
        mount_joint_idx:  int   = DEFAULT_MOUNT_JOINT_IDX,
        max_reach_m:      float = DEFAULT_MAX_REACH_M,
        self_radius_m:    float = DEFAULT_SELF_RADIUS_M,
        reach_margin_m:   float = DEFAULT_REACH_MARGIN_M,
        enabled:          bool  = True,
    ):
        self._chain           = chain
        self._mount_idx       = mount_joint_idx
        self._self_radius_m   = self_radius_m
        self._reach_margin_m  = reach_margin_m
        self._enabled         = enabled

        # Auto-compute max reach from distal link lengths if not provided
        if max_reach_m > 0:
            self._max_reach_m = max_reach_m
            logger.info(f"SweptVolume: max reach set to {max_reach_m:.3f}m (manual)")
        else:
            self._max_reach_m = self._compute_distal_reach()
            logger.info(
                f"SweptVolume: max reach auto-computed as {self._max_reach_m:.3f}m "
                f"from joints {mount_joint_idx+1}→end"
            )

        if not enabled:
            logger.info("SweptVolume: disabled — all points pass through")

    # ------------------------------------------------------------------

    def update(self, q: dict) -> WorkspaceSnapshot:
        """
        Compute workspace geometry for the current joint configuration.
        Call once per frame before filter().

        q — dict of {joint_name: angle} (same format as ego-motion compensator)
        """
        if not self._enabled:
            return WorkspaceSnapshot(
                mount_position = np.zeros(3),
                max_reach_m    = 0.0,
                self_radius_m  = 0.0,
                reach_margin_m = 0.0,
                enabled        = False,
            )

        mount_pos = self._get_mount_position(q)

        return WorkspaceSnapshot(
            mount_position = mount_pos,
            max_reach_m    = self._max_reach_m,
            self_radius_m  = self._self_radius_m,
            reach_margin_m = self._reach_margin_m,
            enabled        = True,
        )

    def filter(self, points: list, q: dict) -> tuple[list, WorkspaceSnapshot]:
        """
        Filter points to only those within the reachable workspace.
        Returns (filtered_points, workspace_snapshot).

        Fail-safe: if disabled or chain is placeholder geometry,
        returns all points unchanged.
        """
        workspace = self.update(q)

        if not workspace.enabled:
            return points, workspace

        # Fail-safe: placeholder chain (all joints at same position)
        # — can't compute meaningful geometry, pass all through
        if self._is_placeholder_chain():
            logger.debug("SweptVolume: placeholder chain — passing all points through")
            return points, workspace

        if not points:
            return points, workspace

        filtered = [
            pt for pt in points
            if workspace.contains(pt.x, pt.y, pt.z)
        ]

        suppressed = len(points) - len(filtered)
        if suppressed > 0:
            logger.debug(
                f"SweptVolume: suppressed {suppressed}/{len(points)} points "
                f"outside workspace (mount={workspace.mount_position}, "
                f"reach={workspace.outer_radius:.2f}m, "
                f"self_excl={workspace.self_radius_m:.2f}m)"
            )

        # Fail-safe: never return empty if we had points coming in
        # If everything was suppressed something is wrong — pass originals
        if not filtered and points:
            logger.warning(
                "SweptVolume: all points suppressed — fail-safe pass-through. "
                "Check mount_joint_idx and max_reach_m parameters."
            )
            return points, workspace

        return filtered, workspace

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _get_mount_position(self, q: dict) -> np.ndarray:
        """
        Walk the kinematic chain to the mount joint and return its
        world-frame origin position.
        """
        joints = self._chain.joints
        T_world = np.eye(4)

        for i, joint in enumerate(joints):
            T_world = T_world @ joint.origin

            if i == self._mount_idx:
                # This is the mount joint — return its world position
                return T_world[:3, 3].copy()

            if joint.type in ('revolute', 'continuous', 'prismatic'):
                angle = q.get(joint.name, 0.0)
                T_world = T_world @ self._chain._joint_transform(joint, angle)

        # Mount index beyond chain length — return end effector position
        logger.warning(
            f"SweptVolume: mount_joint_idx {self._mount_idx} >= chain length "
            f"{len(joints)} — using end effector position"
        )
        T_sensor = T_world @ self._chain.T_mount
        return T_sensor[:3, 3].copy()

    def _compute_distal_reach(self) -> float:
        """
        Sum the translation magnitudes of all joint origins from mount_idx+1
        onward plus the sensor mount offset. This gives the maximum possible
        reach of the distal arm from the mount point.
        """
        joints = self._chain.joints
        total  = 0.0

        for i in range(self._mount_idx + 1, len(joints)):
            origin = joints[i].origin
            t = origin[:3, 3]
            total += float(np.linalg.norm(t))

        # Add sensor mount offset
        t_mount = self._chain.T_mount[:3, 3]
        total += float(np.linalg.norm(t_mount))

        # Floor at a minimum useful value in case geometry is placeholder
        return max(total, 0.30)

    def _is_placeholder_chain(self) -> bool:
        """
        Detect if the chain is using placeholder geometry (all joints
        stacked along Z at 0.1m each). If so, swept-volume math is
        not meaningful — fail-safe to pass all points through.
        """
        joints = self._chain.joints
        if not joints:
            return True

        # Check: are all joint origins identical (placeholder pattern)?
        first_t = joints[0].origin[:3, 3]
        all_same = all(
            np.allclose(j.origin[:3, 3], first_t, atol=1e-3)
            for j in joints[1:]
        )
        # Also check if all translations are the default 0.1m Z
        all_default = all(
            abs(j.origin[2, 3] - 0.1) < 1e-3 and
            abs(j.origin[0, 3]) < 1e-3 and
            abs(j.origin[1, 3]) < 1e-3
            for j in joints
        )
        return all_same or all_default

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def max_reach_m(self) -> float:
        return self._max_reach_m


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import logging
    import sys
    import os
    sys.path.insert(0, os.path.dirname(__file__))
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    # Minimal stubs to test without full ROS environment
    import numpy as np
    from dataclasses import dataclass as dc

    @dc
    class FakeJoint:
        name:   str
        type:   str
        origin: np.ndarray
        axis:   np.ndarray

    def make_T(xyz, rpy=None):
        T = np.eye(4)
        T[:3, 3] = xyz
        return T

    @dc
    class FakePoint:
        x: float; y: float; z: float
        velocity: float = -0.5
        snr: float = 20.0

    # Simulate UR5-like geometry:
    # joints 0-2: base/shoulder/elbow (below mount)
    # joints 3-5: forearm/wrist1/wrist2 (distal from mount at joint 3)
    joints = [
        FakeJoint("joint1", "revolute", make_T([0.0, 0.0, 0.127]), np.array([0,0,1])),
        FakeJoint("joint2", "revolute", make_T([0.0, 0.0, 0.0]),   np.array([0,1,0])),
        FakeJoint("joint3", "revolute", make_T([0.425, 0.0, 0.0]), np.array([0,1,0])),
        FakeJoint("joint4", "revolute", make_T([0.392, 0.0, 0.0]), np.array([0,0,1])),
        FakeJoint("joint5", "revolute", make_T([0.0, 0.109, 0.0]), np.array([0,1,0])),
        FakeJoint("joint6", "revolute", make_T([0.0, -0.093, 0.0]),np.array([0,0,1])),
    ]

    class FakeChain:
        def __init__(self):
            self.joints  = joints
            self.T_mount = make_T([0.0, 0.0, 0.05])
        def _joint_transform(self, joint, q):
            T = np.eye(4)
            return T   # identity for test — zero joint angles

    chain = FakeChain()

    print("=== SweptVolumeClipper test ===\n")

    clipper = SweptVolumeClipper(
        chain           = chain,
        mount_joint_idx = 3,      # sensors on forearm between joint 3 and 4
        self_radius_m   = 0.15,
        reach_margin_m  = 0.20,
    )

    print(f"  max reach (auto):  {clipper.max_reach_m:.3f}m")
    print(f"  outer boundary:    {clipper.max_reach_m + 0.20:.3f}m")
    print(f"  self exclusion:    0.15m\n")

    q = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0,
         "joint4": 0.0, "joint5": 0.0, "joint6": 0.0}

    workspace = clipper.update(q)
    print(f"  mount position:    {workspace.mount_position}")
    print(f"  outer radius:      {workspace.outer_radius:.3f}m\n")

    test_points = [
        # (description, point, expect_pass)
        ("Person 0.5m from mount — inside reach",
         FakePoint(workspace.mount_position[0] + 0.5,
                   workspace.mount_position[1],
                   workspace.mount_position[2]), True),

        ("Person at max reach boundary",
         FakePoint(workspace.mount_position[0] + workspace.outer_radius - 0.01,
                   workspace.mount_position[1],
                   workspace.mount_position[2]), True),

        ("Detection beyond max reach — suppress",
         FakePoint(workspace.mount_position[0] + workspace.outer_radius + 0.5,
                   workspace.mount_position[1],
                   workspace.mount_position[2]), False),

        ("Detection inside self-exclusion — suppress (arm body)",
         FakePoint(workspace.mount_position[0] + 0.05,
                   workspace.mount_position[1],
                   workspace.mount_position[2]), False),

        ("Far wall 5m away — suppress",
         FakePoint(workspace.mount_position[0] + 5.0,
                   workspace.mount_position[1],
                   workspace.mount_position[2]), False),
    ]

    print("--- Point filter tests ---\n")
    all_pass = True
    for desc, pt, expect in test_points:
        result = workspace.contains(pt.x, pt.y, pt.z)
        status = "✅" if result == expect else "❌"
        if result != expect:
            all_pass = False
        print(f"{status}  {desc}")
        print(f"     dist={math.sqrt((pt.x-workspace.mount_position[0])**2 + (pt.y-workspace.mount_position[1])**2 + (pt.z-workspace.mount_position[2])**2):.3f}m "
              f"→ {'PASS' if result else 'SUPPRESS'} (expect {'PASS' if expect else 'SUPPRESS'})\n")

    print(f"  All tests passed: {all_pass}")

    print("\n--- Disabled mode test ---")
    clipper_off = SweptVolumeClipper(chain=chain, enabled=False)
    all_pts = [FakePoint(0.0, 5.0, 0.0), FakePoint(0.0, 0.01, 0.0)]
    filtered, ws = clipper_off.filter(all_pts, q)
    print(f"  disabled: {len(filtered)}/{len(all_pts)} points passed (expect {len(all_pts)})")

    print("\n--- Fail-safe: placeholder chain test ---")
    placeholder_joints = [
        FakeJoint(f"joint{i}", "revolute",
                  make_T([0.0, 0.0, 0.1]), np.array([0,0,1]))
        for i in range(6)
    ]
    class PlaceholderChain:
        joints  = placeholder_joints
        T_mount = make_T([0.0, 0.0, 0.05])
        def _joint_transform(self, j, q): return np.eye(4)

    clipper_ph = SweptVolumeClipper(chain=PlaceholderChain(), mount_joint_idx=3)
    filtered_ph, _ = clipper_ph.filter(all_pts, q)
    print(f"  placeholder: {len(filtered_ph)}/{len(all_pts)} points passed (expect {len(all_pts)} — fail-safe)")
