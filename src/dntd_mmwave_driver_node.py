"""
dntd_mmwave_driver_node.py
DNTD Dynamics — mmWave UART → ROS 2 Driver
Uses MmwaveReader (uart_reader.py) directly — no adapter layer.

Publishes:
  mmwave/raw_points   (sensor_msgs/PointCloud2)
  mmwave/diagnostics  (diagnostic_msgs/DiagnosticStatus)
"""

import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import sensor_msgs_py.point_cloud2 as pc2

from uart_reader import MmwaveReader

POINT_FIELDS = [
    PointField(name='x',        offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',        offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',        offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='snr',      offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='noise',    offset=20, datatype=PointField.FLOAT32, count=1),
]


class DntdMmwaveDriverNode(Node):

    def __init__(self):
        super().__init__('dntd_mmwave_driver')

        self.declare_parameter('cli_port',        '/dev/ttyUSB0')
        self.declare_parameter('cli_baud',        115200)
        self.declare_parameter('data_port',       '/dev/ttyUSB1')
        self.declare_parameter('data_baud',       921600)
        self.declare_parameter('config_file',
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'configs', 'profile_AOP.cfg'))
        self.declare_parameter('sensor_frame_id', 'mmwave_sensor')
        self.declare_parameter('publish_hz',      10.0)
        self.declare_parameter('send_config',     True)
        self.declare_parameter('config_retry',    True)
        self.declare_parameter('sensor_model',    'iwr6843aop')

        self._frame_id   = self.get_parameter('sensor_frame_id').value
        self._publish_hz = self.get_parameter('publish_hz').value
        cfg_file         = self.get_parameter('config_file').value
        send_cfg         = self.get_parameter('send_config').value
        retry            = self.get_parameter('config_retry').value

        # Diagnostics counters
        self._frames_published = 0
        self._last_frame_time  = None
        self._sensor_ok        = False
        self._diag_lock        = threading.Lock()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # Publishers
        self._pub_cloud = self.create_publisher(PointCloud2, '/dntd/mmwave/raw_points', sensor_qos)
        self._pub_diag  = self.create_publisher(DiagnosticStatus, '/dntd/mmwave/diagnostics', 10)

        # MmwaveReader
        self._reader = MmwaveReader(
            cli_port  = self.get_parameter('cli_port').value,
            data_port = self.get_parameter('data_port').value,
            cli_baud  = self.get_parameter('cli_baud').value,
            data_baud = self.get_parameter('data_baud').value,
        )

        # Setup runs in background thread so node stays responsive during config send
        self._ready = threading.Event()
        threading.Thread(
            target=self._setup,
            args=(cfg_file, send_cfg, retry),
            daemon=True,
            name='mmwave_setup',
        ).start()

        # Timers
        self.create_timer(1.0 / self._publish_hz, self._publish_loop)
        self.create_timer(2.0, self._publish_diagnostics)

        self.get_logger().info(
            f"DNTD mmWave driver started\n"
            f"  model      : {self.get_parameter('sensor_model').value}\n"
            f"  CLI port   : {self.get_parameter('cli_port').value}\n"
            f"  data port  : {self.get_parameter('data_port').value}\n"
            f"  config     : {cfg_file}\n"
            f"  frame_id   : {self._frame_id}\n"
            f"  send_config: {send_cfg}"
        )

    # ------------------------------------------------------------------

    def _setup(self, cfg_file, send_cfg, retry):
        """Background thread: send config then start UART reader."""
        if send_cfg:
            self.get_logger().info(f"Sending config: {cfg_file}")
            errors = self._reader.send_config(cfg_file)
            if errors:
                for line, resp in errors:
                    self.get_logger().error(f"Config error on '{line}': {resp}")
                if retry:
                    self.get_logger().warning("Retrying config...")
                    errors = self._reader.send_config(cfg_file)
                    if errors:
                        self.get_logger().error("Config retry failed — sensor may not stream")
                    else:
                        self.get_logger().info("Config retry succeeded")
            else:
                self.get_logger().info("Config sent cleanly")
        else:
            self.get_logger().info("send_config=false — reading existing stream")

        self._reader.start()
        self._ready.set()
        self.get_logger().info("UART reader running — frames incoming")

    # ------------------------------------------------------------------

    def _publish_loop(self):
        """Drain all available frames from MmwaveReader queue each timer tick."""
        if not self._ready.is_set():
            return

        while True:
            frame = self._reader.get_frame(timeout=0.0)
            if frame is None:
                break

            cloud_msg = self._build_pointcloud2(frame)
            if cloud_msg is None:
                continue

            self._pub_cloud.publish(cloud_msg)

            with self._diag_lock:
                self._frames_published += 1
                self._last_frame_time  = time.monotonic()
                self._sensor_ok        = True

    # ------------------------------------------------------------------

    def _publish_diagnostics(self):
        with self._diag_lock:
            published = self._frames_published
            last_t    = self._last_frame_time
            ok        = self._sensor_ok

        age   = time.monotonic() - last_t if last_t else float('inf')
        stale = age > (2.0 / max(self._publish_hz, 1.0))

        msg             = DiagnosticStatus()
        msg.name        = f"dntd_mmwave/{self.get_name()}"
        msg.hardware_id = self.get_parameter('sensor_model').value

        if not ok:
            msg.level   = DiagnosticStatus.WARN
            msg.message = "No frames yet — config may still be sending"
        elif stale:
            msg.level   = DiagnosticStatus.WARN
            msg.message = f"No frames for {age:.1f}s"
        else:
            msg.level   = DiagnosticStatus.OK
            msg.message = "Streaming normally"

        msg.values = [
            KeyValue(key='frames_published', value=str(published)),
            KeyValue(key='last_frame_age_s', value=f"{age:.2f}"),
            KeyValue(key='queue_depth',
                     value=str(self._reader.frame_queue.qsize())),
        ]
        self._pub_diag.publish(msg)

    # ------------------------------------------------------------------

    def _build_pointcloud2(self, frame) -> PointCloud2 | None:
        if not frame.points:
            return None

        header          = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id

        rows = [
            [pt.x, pt.y, pt.z, pt.velocity, pt.snr, pt.noise]
            for pt in frame.points
        ]
        return pc2.create_cloud(header, POINT_FIELDS, rows)

    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Shutting down mmWave driver...")
        self._reader.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DntdMmwaveDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
