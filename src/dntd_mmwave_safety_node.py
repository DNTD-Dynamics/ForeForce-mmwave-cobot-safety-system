"""
dntd_mmwave_safety_node.py
DNTD Dynamics — mmWave Arm Safety System
ROS 2 node: ego-motion compensation + zone classification

Subscribes:
  /joint_states                    (sensor_msgs/JointState)
  /dntd/mmwave/raw_points          (sensor_msgs/PointCloud2) — from mmwave_driver node
  /dntd/safety_resume              (std_msgs/Bool)           — explicit resume after fault

Publishes:
  /dntd/safety_zone                (std_msgs/String)         — CLEAR / CAUTION / STOP
  /dntd/safety_fault               (std_msgs/String)         — fault reason or empty
  /dntd/heartbeat                  (std_msgs/Header)         — watchdog pulse
  /dntd/compensated_points         (sensor_msgs/PointCloud2) — world-frame point cloud

Config (YAML, see dntd_mmwave_config.yaml):
  sensor_mount_link       — URDF link the sensor is rigidly attached to
  sensor_mount_transform  — xyz + rpy offset from that link to sensor origin
  joint_names             — ordered list of joints in the kinematic chain
  interpolate_joint_states — true/false
  joint_states_timeout_s  — seconds before declaring joint_states fault
  stop_range_m            — hard stop radius
  caution_range_m         — slow-down radius
  fast_approach_mps       — approach velocity that triggers emergency stop
  heartbeat_hz            — watchdog pulse rate
"""

import numpy as np
import threading
import time
import logging
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String, Bool, Header
import sensor_msgs_py.point_cloud2 as pc2

from zone_logic import ZoneClassifier, ZoneOutputs, DetectedPoint, ZoneState
from background_model import BackgroundModel
from cluster import ClusterBuilder
from classifier import MicroDopplerClassifier

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Transform utilities (no tf2 dependency — readable and portable)
# ---------------------------------------------------------------------------

def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=float)

def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=float)

def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=float)

def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Intrinsic XYZ RPY → rotation matrix."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

def make_transform(xyz, rpy) -> np.ndarray:
    """4×4 homogeneous transform from xyz list and rpy list."""
    T = np.eye(4)
    T[:3,:3] = rpy_to_rot(*rpy)
    T[:3, 3] = xyz
    return T


# ---------------------------------------------------------------------------
# Kinematic chain
# ---------------------------------------------------------------------------

@dataclass
class Joint:
    """One joint in the kinematic chain, parsed from URDF/config."""
    name:   str
    type:   str           # 'revolute', 'continuous', 'prismatic', 'fixed'
    origin: np.ndarray    # 4×4 transform from parent link to joint frame (constant)
    axis:   np.ndarray    # unit axis vector in joint frame (3,)


class KinematicChain:
    """
    Minimal forward kinematics for ego-motion compensation.
    Reads joint geometry from config — no full URDF parser required.
    Users supply joint origins and axes in dntd_mmwave_config.yaml.

    For each radar frame the chain computes:
      1. World-frame position of the sensor origin
      2. Geometric Jacobian (linear velocity columns only) of the sensor
         with respect to all active joints
    """

    def __init__(self, joints: list[Joint], sensor_mount_transform: np.ndarray):
        """
        joints                 — ordered base→sensor list of Joint objects
        sensor_mount_transform — 4×4 offset from last link to sensor origin
        """
        self.joints  = joints
        self.T_mount = sensor_mount_transform   # constant offset to sensor

    def forward(self, q: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute sensor position and Jacobian given joint positions.

        q — dict of {joint_name: angle_or_displacement}

        Returns:
          p_sensor  — (3,) sensor origin in world frame
          J         — (3, n_active) linear velocity Jacobian
                      columns ordered to match self.active_joints
        """
        # Walk the chain: accumulate transforms base → sensor
        T_world = np.eye(4)
        link_transforms = []   # world-frame transform at each joint origin

        for joint in self.joints:
            # Fixed offset from parent link to this joint's origin
            T_world = T_world @ joint.origin

            link_transforms.append(T_world.copy())

            if joint.type in ('revolute', 'continuous', 'prismatic'):
                angle = q.get(joint.name, 0.0)
                T_world = T_world @ self._joint_transform(joint, angle)

        # Apply sensor mount offset
        T_sensor = T_world @ self.T_mount
        p_sensor = T_sensor[:3, 3]

        # Build Jacobian columns
        J_cols = []
        for i, joint in enumerate(self.joints):
            if joint.type not in ('revolute', 'continuous', 'prismatic'):
                continue

            T_joint = link_transforms[i]
            p_joint = T_joint[:3, 3]                       # joint origin in world
            z_joint = T_joint[:3,:3] @ joint.axis          # joint axis in world

            if joint.type in ('revolute', 'continuous'):
                # Linear velocity contribution: z × (p_sensor - p_joint)
                col = np.cross(z_joint, p_sensor - p_joint)
            else:
                # Prismatic: linear velocity along joint axis
                col = z_joint

            J_cols.append(col)

        J = np.column_stack(J_cols) if J_cols else np.zeros((3, 0))
        return p_sensor, J

    @staticmethod
    def _joint_transform(joint: Joint, q: float) -> np.ndarray:
        """4×4 transform for joint displacement q."""
        T = np.eye(4)
        if joint.type in ('revolute', 'continuous'):
            # Rodrigues rotation around joint axis
            axis = joint.axis / np.linalg.norm(joint.axis)
            c, s = np.cos(q), np.sin(q)
            K = np.array([
                [ 0,        -axis[2],  axis[1]],
                [ axis[2],  0,        -axis[0]],
                [-axis[1],  axis[0],  0       ],
            ])
            T[:3,:3] = np.eye(3)*c + (1-c)*np.outer(axis,axis) + s*K
        else:
            # Prismatic: translate along axis
            T[:3, 3] = joint.axis * q
        return T

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.joints
                if j.type in ('revolute', 'continuous', 'prismatic')]


# ---------------------------------------------------------------------------
# Joint state buffer with interpolation
# ---------------------------------------------------------------------------

class JointStateBuffer:
    """
    Thread-safe ring buffer of JointState messages.
    Provides interpolated joint positions/velocities at any timestamp.
    Falls back to nearest-neighbor if interpolation is disabled.
    """

    BUFFER_SIZE = 50   # ~5 seconds at 10Hz joint_states

    def __init__(self, joint_names: list[str], interpolate: bool = True):
        self.joint_names  = joint_names
        self.interpolate  = interpolate
        self._lock        = threading.Lock()
        self._times: list[float]            = []
        self._positions: list[dict[str, float]] = []
        self._velocities: list[dict[str, float]] = []

    def add(self, msg: JointState, stamp: float):
        with self._lock:
            pos = dict(zip(msg.name, msg.position))
            vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
            self._times.append(stamp)
            self._positions.append(pos)
            self._velocities.append(vel)
            if len(self._times) > self.BUFFER_SIZE:
                self._times.pop(0)
                self._positions.pop(0)
                self._velocities.pop(0)

    def get(self, t: float) -> Optional[tuple[dict, dict]]:
        """
        Return (positions, velocities) interpolated at time t.
        Returns None if buffer is empty or t is out of range.
        """
        with self._lock:
            if not self._times:
                return None

            times = self._times
            # Clamp to buffer range
            if t <= times[0]:
                return self._positions[0], self._velocities[0]
            if t >= times[-1]:
                return self._positions[-1], self._velocities[-1]

            if not self.interpolate:
                # Nearest neighbor
                idx = min(range(len(times)), key=lambda i: abs(times[i] - t))
                return self._positions[idx], self._velocities[idx]

            # Linear interpolation between bracketing samples
            for i in range(len(times) - 1):
                if times[i] <= t <= times[i+1]:
                    alpha = (t - times[i]) / (times[i+1] - times[i] + 1e-9)
                    pos = self._interp_dicts(
                        self._positions[i], self._positions[i+1], alpha)
                    vel = self._interp_dicts(
                        self._velocities[i], self._velocities[i+1], alpha)
                    return pos, vel

            return self._positions[-1], self._velocities[-1]

    def latest_stamp(self) -> Optional[float]:
        with self._lock:
            return self._times[-1] if self._times else None

    @staticmethod
    def _interp_dicts(a: dict, b: dict, alpha: float) -> dict:
        keys = set(a) | set(b)
        return {k: a.get(k, 0.0) * (1-alpha) + b.get(k, 0.0) * alpha
                for k in keys}


# ---------------------------------------------------------------------------
# Ego-motion compensator
# ---------------------------------------------------------------------------

class EgoMotionCompensator:
    """
    Given a raw radar point cloud (sensor frame) and current joint state,
    returns a compensated point cloud where each point's velocity has
    the sensor's own motion subtracted out.

    A point with compensated velocity near zero is static in the world.
    A point with negative compensated velocity is approaching in the world.
    """

    def __init__(self, chain: KinematicChain):
        self.chain = chain

    def compensate(
        self,
        points:     list[DetectedPoint],
        q:          dict[str, float],
        q_dot:      dict[str, float],
    ) -> list[DetectedPoint]:
        """
        points  — raw DetectedPoint list from tlv_parser (sensor frame)
        q       — joint positions {name: value}
        q_dot   — joint velocities {name: value}

        Returns new DetectedPoint list with velocity field corrected.
        """
        if not points:
            return points

        # Compute sensor velocity in world frame
        _, J = self.chain.forward(q)

        if J.shape[1] == 0:
            # No active joints — sensor is stationary (fixed mount case)
            return points

        # q_dot as ordered vector matching Jacobian columns
        q_dot_vec = np.array([
            q_dot.get(name, 0.0)
            for name in self.chain.joint_names
        ])

        v_sensor = J @ q_dot_vec    # (3,) sensor velocity in world frame

        # For each point, project sensor velocity onto the point's
        # unit range vector — this is the doppler contribution from
        # sensor motion alone
        compensated = []
        for pt in points:
            p_vec = np.array([pt.x, pt.y, pt.z])
            r = np.linalg.norm(p_vec)
            if r < 1e-6:
                compensated.append(pt)
                continue

            unit_r = p_vec / r
            # Sensor motion component along the range direction
            v_ego_radial = float(np.dot(v_sensor, unit_r))

            # Subtract: positive ego-motion toward point inflates approach
            # velocity — remove it
            v_corrected = pt.velocity - v_ego_radial

            compensated.append(DetectedPoint(
                x=pt.x, y=pt.y, z=pt.z,
                velocity=v_corrected,
                snr=pt.snr,
            ))

        return compensated


# ---------------------------------------------------------------------------
# ROS 2 safety node
# ---------------------------------------------------------------------------

class DntdMmwaveSafetyNode(Node):
    """
    Main ROS 2 node. Wires together:
      JointStateBuffer → EgoMotionCompensator → ZoneClassifier → outputs
    """

    def __init__(self):
        super().__init__('dntd_mmwave_safety')

        # --- Declare parameters (all tunable from YAML or command line) ---
        self.declare_parameter('sensor_mount_link',        'tool0')
        self.declare_parameter('sensor_mount_xyz',         [0.0, 0.0, 0.05])
        self.declare_parameter('sensor_mount_rpy',         [0.0, 0.0, 0.0])
        self.declare_parameter('joint_names',              ['joint1','joint2','joint3',
                                                            'joint4','joint5','joint6'])
        self.declare_parameter('interpolate_joint_states', True)
        self.declare_parameter('joint_states_timeout_s',   0.5)
        self.declare_parameter('stop_range_m',             0.5)
        self.declare_parameter('caution_range_m',          1.2)
        self.declare_parameter('fast_approach_mps',        -0.8)
        self.declare_parameter('static_filter_mps',        0.3)
        self.declare_parameter('min_snr_db',               8.0)
        self.declare_parameter('heartbeat_hz',             5.0)
        self.declare_parameter('output_serial_port',       '')
        self.declare_parameter('output_use_gpio',          False)
        self.declare_parameter('output_mqtt_broker',       '')

        self.declare_parameter('background_voxel_size_m', 0.10)
        self.declare_parameter('background_learning_s', 15.0)
        self.declare_parameter('background_hit_threshold', 0.30)

        # Micro-doppler classifier parameters
        self.declare_parameter('classifier_enabled',             True)
        self.declare_parameter('classifier_min_points',          2)
        self.declare_parameter('classifier_velocity_spread_min', 0.08)
        self.declare_parameter('classifier_height_span_min',     0.10)
        self.declare_parameter('classifier_point_count_min',     3)
        self.declare_parameter('classifier_score_threshold',     2)
        self.declare_parameter('classifier_log_enabled',         True)
        self.declare_parameter('classifier_eps_m',               0.40)
	
        # --- Load parameters ---
        p = self._params()

        # --- Kinematic chain (populated from config) ---
        # Users extend this by adding joints in their YAML.
        # Default is a placeholder 6DOF chain — replace with real geometry.
        self._chain = self._build_chain_from_params(p)

        # --- Sub-components ---
        self._js_buffer = JointStateBuffer(
            p['joint_names'],
            interpolate=p['interpolate_joint_states'],
        )
        self._compensator = EgoMotionCompensator(self._chain)
        self._background = BackgroundModel(
            voxel_size           = self.get_parameter('background_voxel_size_m').value,
            learning_duration_s  = self.get_parameter('background_learning_s').value,
            hit_threshold        = self.get_parameter('background_hit_threshold').value,
	)
        # Cluster builder and micro-doppler classifier
        self._cluster_builder = ClusterBuilder(
            eps_m      = self.get_parameter('classifier_eps_m').value,
        )
        clf_thresholds = {
            'min_points_to_classify':      self.get_parameter('classifier_min_points').value,
            'person_velocity_spread_min':  self.get_parameter('classifier_velocity_spread_min').value,
            'person_height_span_min':      self.get_parameter('classifier_height_span_min').value,
            'person_point_count_min':      self.get_parameter('classifier_point_count_min').value,
            'person_score_threshold':      self.get_parameter('classifier_score_threshold').value,
        }
        self._micro_doppler = MicroDopplerClassifier(
            thresholds     = clf_thresholds,
            enable_logging = self.get_parameter('classifier_log_enabled').value,
        )
        self._classifier_enabled = self.get_parameter('classifier_enabled').value

        self._classifier  = ZoneClassifier(
            stop_range    = p['stop_range_m'],
            caution_range = p['caution_range_m'],
            fast_approach = p['fast_approach_mps'],
            static_filter = p['static_filter_mps'],
            min_snr       = p['min_snr_db'],
        )
        self._outputs = ZoneOutputs(
            serial_port = p['output_serial_port'] or None,
            use_gpio    = p['output_use_gpio'],
            mqtt_broker = p['output_mqtt_broker'] or None,
        )

        # --- Fault state ---
        self._fault_active  = False
        self._fault_reason  = ''
        self._resume_armed  = False   # True after controller sends resume
        self._last_js_stamp = None
        self._js_timeout    = p['joint_states_timeout_s']
        self._state_lock    = threading.Lock()

        # --- QoS: best-effort for sensor data, reliable for safety commands ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # --- Subscribers ---
        self.create_subscription(
            JointState, '/joint_states',
            self._on_joint_states, 10)
        self.create_subscription(
            PointCloud2, '/dntd/mmwave/raw_points',
            self._on_raw_points, sensor_qos)
        self.create_subscription(
            Bool, '/dntd/safety_resume',
            self._on_resume, reliable_qos)
        self.create_subscription(
            Bool, '/dntd/relearn_background',
            self._on_relearn, reliable_qos)

        # --- Publishers ---
        self._pub_zone  = self.create_publisher(
            String, '/dntd/safety_zone', reliable_qos)
        self._pub_fault = self.create_publisher(
            String, '/dntd/safety_fault', reliable_qos)
        self._pub_hb    = self.create_publisher(
            Header, '/dntd/heartbeat', 10)
        self._pub_comp  = self.create_publisher(
            PointCloud2, '/dntd/compensated_points', sensor_qos)

        # --- Timers ---
        hb_period = 1.0 / max(p['heartbeat_hz'], 0.1)
        self.create_timer(hb_period,           self._publish_heartbeat)
        self.create_timer(self._js_timeout / 2, self._check_joint_states_watchdog)

        self.get_logger().info(
            "DNTD mmWave safety node started\n"
            f"  sensor mount link : {p['sensor_mount_link']}\n"
            f"  joints            : {p['joint_names']}\n"
            f"  stop / caution    : {p['stop_range_m']}m / {p['caution_range_m']}m\n"
            f"  fast approach     : {p['fast_approach_mps']} m/s\n"
            f"  heartbeat         : {p['heartbeat_hz']} Hz\n"
            f"  classifier        : {'enabled' if self._classifier_enabled else 'disabled'}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        stamp = self._msg_stamp(msg.header)
        self._js_buffer.add(msg, stamp)
        with self._state_lock:
            self._last_js_stamp = time.monotonic()
            # If we were in a joint_states fault and resume was armed,
            # clear the fault now that joint_states has recovered
            if self._fault_active and 'joint_states' in self._fault_reason:
                if self._resume_armed:
                    self._clear_fault()

    def _on_raw_points(self, msg: PointCloud2):
        """Main processing pipeline per radar frame."""
        frame_stamp = self._msg_stamp(msg.header)

        # Check fault state — if faulted, publish STOP and return
        with self._state_lock:
            if self._fault_active:
                self._publish_zone('STOP', fault=self._fault_reason)
                return

        # Get interpolated joint state at frame timestamp
        js = self._js_buffer.get(frame_stamp)
        if js is None:
            self._raise_fault('joint_states: no data received yet')
            return

        q, q_dot = js

        # Decode PointCloud2 → DetectedPoint list
        raw_points = self._decode_pointcloud(msg)

        # Ego-motion compensation
        compensated = self._compensator.compensate(raw_points, q, q_dot)

        # Background learning / masking
        self._background.observe(compensated)

        if self._background.is_learning:
            stats = self._background.get_stats()
            self._publish_zone('STOP',
                fault=f"Learning background ({stats.seconds_remaining:.0f}s left)")
            return

        # Mask out background returns — only novel points reach the classifier
        novel_points = self._background.filter_novel(compensated)

        # Publish compensated cloud for RViz / downstream nodes
        self._pub_comp.publish(
            self._encode_pointcloud(novel_points, msg.header))

        # Micro-doppler classification — group into clusters, label each
        # PERSON/UNKNOWN pass through (fail-safe), OBJECT is suppressed
        if self._classifier_enabled and novel_points:
            clusters      = self._cluster_builder.update(novel_points)
            safe_clusters = self._micro_doppler.filter_person_clusters(clusters)
            # Reconstruct point list from safe clusters only
            # Zone classifier still works on DetectedPoints — unchanged interface
            safe_points = [
                pt for pt in novel_points
                if any(
                    abs(pt.x - c.centroid_x) <= 0.5 and
                    abs(pt.y - c.centroid_y) <= 0.5 and
                    abs(pt.z - c.centroid_z) <= 0.5
                    for c in safe_clusters
                )
            ] if safe_clusters else []
            # Fail-safe: if classifier suppressed everything but we had novel
            # points, pass originals through rather than falsely reporting CLEAR
            zone_points = safe_points if safe_points else novel_points
        else:
            zone_points = novel_points

        # Zone classification on classifier-filtered points
        state = self._classifier.update_frame(zone_points)

        # Publish zone + downstream outputs
        self._publish_zone(state.zone)
        self._outputs.publish(state)

    def _on_resume(self, msg: Bool):
        """
        Controller sends True to arm the resume.
        Fault clears on next valid joint_states message (for joint_states faults)
        or immediately (for other faults) if zone is CLEAR.
        """
        if not msg.data:
            return
        with self._state_lock:
            if not self._fault_active:
                return
            self._resume_armed = True
            self.get_logger().info(
                "Resume armed — fault will clear on next valid joint_states "
                "if zone is CLEAR"
            )
            # For non-joint_states faults, clear immediately if zone allows
            if 'joint_states' not in self._fault_reason:
                current_zone = self._classifier.get_state().zone
                if current_zone == 'CLEAR':
                    self._clear_fault()

    def _on_relearn(self, msg: Bool):
        if msg.data:
            self._background.start_relearn()
            self.get_logger().info("Background relearn requested")

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _check_joint_states_watchdog(self):
        with self._state_lock:
            if self._last_js_stamp is None:
                return  # Haven't received any yet — startup grace period
            age = time.monotonic() - self._last_js_stamp
            if age > self._js_timeout and not self._fault_active:
                self._raise_fault(
                    f'joint_states: no message for {age:.2f}s '
                    f'(timeout={self._js_timeout}s)'
                )

    # ------------------------------------------------------------------
    # Fault management
    # ------------------------------------------------------------------

    def _raise_fault(self, reason: str):
        """Call with _state_lock held."""
        if self._fault_active:
            return
        self._fault_active  = True
        self._fault_reason  = reason
        self._resume_armed  = False
        fault_msg           = String(data=reason)
        self._pub_fault.publish(fault_msg)
        self._publish_zone('STOP', fault=reason)
        self.get_logger().error(f"SAFETY FAULT: {reason}")

    def _clear_fault(self):
        """Call with _state_lock held."""
        self.get_logger().info(
            f"Fault cleared: '{self._fault_reason}' — resuming normal operation"
        )
        self._fault_active  = False
        self._fault_reason  = ''
        self._resume_armed  = False
        self._pub_fault.publish(String(data=''))

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_zone(self, zone: str, fault: str = ''):
        msg = String(data=zone)
        self._pub_zone.publish(msg)
        if fault:
            self._pub_fault.publish(String(data=fault))

    def _publish_heartbeat(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'dntd_mmwave_safety'
        self._pub_hb.publish(h)

    # ------------------------------------------------------------------
    # PointCloud2 encode / decode
    # ------------------------------------------------------------------

    @staticmethod
    def _decode_pointcloud(msg: PointCloud2) -> list[DetectedPoint]:
        """Convert ROS PointCloud2 → DetectedPoint list."""
        points = []
        for p in pc2.read_points(msg, field_names=('x','y','z','velocity','snr'),
                                  skip_nans=True):
            points.append(DetectedPoint(
                x=float(p[0]), y=float(p[1]), z=float(p[2]),
                velocity=float(p[3]),
                snr=float(p[4]) if len(p) > 4 else 15.0,
            ))
        return points

    @staticmethod
    def _encode_pointcloud(
        points: list[DetectedPoint],
        header: Header,
    ) -> PointCloud2:
        """Convert DetectedPoint list → ROS PointCloud2."""
        fields = [
            PointField(name='x',        offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',        offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',        offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='snr',      offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        data = []
        for pt in points:
            data.append([pt.x, pt.y, pt.z, pt.velocity, pt.snr])
        return pc2.create_cloud(header, fields, data)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _params(self) -> dict:
        return {
            'sensor_mount_link':        self.get_parameter('sensor_mount_link').value,
            'sensor_mount_xyz':         self.get_parameter('sensor_mount_xyz').value,
            'sensor_mount_rpy':         self.get_parameter('sensor_mount_rpy').value,
            'joint_names':              self.get_parameter('joint_names').value,
            'interpolate_joint_states': self.get_parameter('interpolate_joint_states').value,
            'joint_states_timeout_s':   self.get_parameter('joint_states_timeout_s').value,
            'stop_range_m':             self.get_parameter('stop_range_m').value,
            'caution_range_m':          self.get_parameter('caution_range_m').value,
            'fast_approach_mps':        self.get_parameter('fast_approach_mps').value,
            'static_filter_mps':        self.get_parameter('static_filter_mps').value,
            'min_snr_db':               self.get_parameter('min_snr_db').value,
            'heartbeat_hz':             self.get_parameter('heartbeat_hz').value,
            'output_serial_port':       self.get_parameter('output_serial_port').value,
            'output_use_gpio':          self.get_parameter('output_use_gpio').value,
            'output_mqtt_broker':       self.get_parameter('output_mqtt_broker').value,
        }

    @staticmethod
    def _build_chain_from_params(p: dict) -> KinematicChain:
        """
        Builds a placeholder kinematic chain from parameter names.
        Users replace joint origins/axes in their YAML config.
        See dntd_mmwave_config.yaml for the full editable format.

        Default: 6 joints along Z axis, 0.1m link length each.
        Replace with real DH parameters or URDF-extracted transforms.
        """
        joints = []
        link_length = 0.1   # placeholder — override in YAML

        for i, name in enumerate(p['joint_names']):
            origin = make_transform(
                xyz=[0.0, 0.0, link_length],
                rpy=[0.0, 0.0, 0.0],
            )
            joints.append(Joint(
                name   = name,
                type   = 'revolute',
                origin = origin,
                axis   = np.array([0.0, 0.0, 1.0]),
            ))

        sensor_T = make_transform(
            xyz=p['sensor_mount_xyz'],
            rpy=p['sensor_mount_rpy'],
        )
        return KinematicChain(joints, sensor_T)

    @staticmethod
    def _msg_stamp(header: Header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DntdMmwaveSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._outputs.cleanup()
        node._micro_doppler.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
