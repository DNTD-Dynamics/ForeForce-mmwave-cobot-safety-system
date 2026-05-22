#!/usr/bin/env python3
"""
arm_controller_node.py — RadarGuard ESP32 ↔ ROS 2 bridge

Reads joint angles from ESP32 serial stream and publishes to /joint_states.
Forwards ROS 2 motion commands (JOG, SWEEP, HOME) back to ESP32 over serial.

Serial protocol (ESP32 → Jetson):
    J,<a0>,<a1>,<a2>,<a3>,<a4>,<a5>   angles in radians, 10Hz

ROS 2 topics:
    Publishes:  /joint_states  (sensor_msgs/JointState)
    Subscribes: /arm_cmd       (std_msgs/String)  — forwarded raw to ESP32

/arm_cmd examples (publish from terminal):
    ros2 topic pub --once /arm_cmd std_msgs/String "data: 'JOG 0 400'"
    ros2 topic pub --once /arm_cmd std_msgs/String "data: 'SWEEP 0 800 5'"
    ros2 topic pub --once /arm_cmd std_msgs/String "data: 'HOME'"
    ros2 topic pub --once /arm_cmd std_msgs/String "data: 'STATUS'"
    ros2 topic pub --once /arm_cmd std_msgs/String "data: 'SPEED ALL 600'"
    ros2 topic pub --once /arm_cmd std_msgs/String "data: 'STOP'"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import serial
import serial.tools.list_ports
import threading
import time

# ---------------------------------------------------------------------------
# Config — override via ROS 2 params or edit here
# ---------------------------------------------------------------------------
DEFAULT_PORT      = "/dev/ttyUSB2"   # adjust — use `ls /dev/ttyUSB*` to find
DEFAULT_BAUD      = 115200
RECONNECT_DELAY_S = 3.0
PUBLISH_FRAME     = "base_link"

JOINT_NAMES = [
    "joint_base",
    "joint_shoulder",
    "joint_elbow",
    "joint_forearm",
    "joint_wrist1",
    "joint_wrist2",
]


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__("arm_controller_node")

        # ROS 2 params
        self.declare_parameter("serial_port", DEFAULT_PORT)
        self.declare_parameter("baud_rate",   DEFAULT_BAUD)

        self._port = self.get_parameter("serial_port").get_parameter_value().string_value
        self._baud = self.get_parameter("baud_rate").get_parameter_value().integer_value

        # Publisher
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Subscriber — raw command strings forwarded to ESP32
        self._cmd_sub = self.create_subscription(
            String, "/arm_cmd", self._cmd_callback, 10
        )

        # Serial state
        self._ser    = None
        self._lock   = threading.Lock()
        self._angles = [0.0] * 6
        self._last_rx = 0.0
        self._connected = False

        # Watchdog — warn if no data for 2s
        self.create_timer(2.0, self._watchdog)

        # Serial reader thread
        self._running = True
        self._thread  = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"arm_controller_node started — port={self._port} baud={self._baud}"
        )
        self.get_logger().info(
            "Send commands via:  ros2 topic pub --once /arm_cmd std_msgs/String \"data: 'HOME'\""
        )

    # -----------------------------------------------------------------------
    # Serial read loop (runs in background thread)
    # -----------------------------------------------------------------------

    def _serial_loop(self):
        while self._running:
            try:
                self.get_logger().info(f"Connecting to ESP32 on {self._port}...")
                ser = serial.Serial(self._port, self._baud, timeout=1.0)
                with self._lock:
                    self._ser       = ser
                    self._connected = True
                self.get_logger().info("ESP32 connected.")

                while self._running:
                    line = ser.readline()
                    if not line:
                        continue
                    try:
                        decoded = line.decode("utf-8", errors="ignore").strip()
                    except Exception:
                        continue

                    # Joint state line: J,a0,a1,a2,a3,a4,a5
                    if decoded.startswith("J,"):
                        parts = decoded.split(",")
                        if len(parts) == 7:
                            try:
                                angles = [float(p) for p in parts[1:]]
                                with self._lock:
                                    self._angles  = angles
                                    self._last_rx = time.time()
                                self._publish(angles)
                            except ValueError:
                                pass

                    # Print any other lines (STATUS, CMD echoes, warnings)
                    else:
                        if decoded:
                            self.get_logger().info(f"[ESP32] {decoded}")

            except serial.SerialException as e:
                self.get_logger().warn(f"Serial error: {e} — retrying in {RECONNECT_DELAY_S}s")
                with self._lock:
                    self._ser       = None
                    self._connected = False
            except Exception as e:
                self.get_logger().error(f"Unexpected serial error: {e}")
                with self._lock:
                    self._ser       = None
                    self._connected = False

            time.sleep(RECONNECT_DELAY_S)

    # -----------------------------------------------------------------------
    # Publish /joint_states
    # -----------------------------------------------------------------------

    def _publish(self, angles):
        msg             = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = PUBLISH_FRAME
        msg.name         = JOINT_NAMES
        msg.position     = angles
        msg.velocity     = [0.0] * 6
        msg.effort       = [0.0] * 6
        self._js_pub.publish(msg)

    # -----------------------------------------------------------------------
    # /arm_cmd subscriber — forward raw string to ESP32
    # -----------------------------------------------------------------------

    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        if not cmd:
            return
        self.get_logger().info(f"Forwarding command: {cmd}")
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write((cmd + "\n").encode("utf-8"))
                except serial.SerialException as e:
                    self.get_logger().error(f"Failed to send command: {e}")
            else:
                self.get_logger().warn("ESP32 not connected — command dropped")

    # -----------------------------------------------------------------------
    # Watchdog
    # -----------------------------------------------------------------------

    def _watchdog(self):
        with self._lock:
            connected = self._connected
            last_rx   = self._last_rx

        if not connected:
            self.get_logger().warn("ESP32 not connected — /joint_states not publishing")
            return

        age = time.time() - last_rx
        if last_rx > 0 and age > 2.0:
            self.get_logger().warn(
                f"No joint data for {age:.1f}s — check ESP32 serial output"
            )

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        with self._lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
