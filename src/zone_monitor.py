#!/usr/bin/env python3
"""
zone_monitor.py — timestamped live view of /dntd/safety_zone, for filming.

Run directly in its own terminal:
    python3 zone_monitor.py

Prints one line per zone transition with a wall-clock timestamp, so
footage of this terminal is its own proof of real-time operation rather
than needing a separate clock in frame.

Subscribes directly with rclpy instead of piping `ros2 topic echo`
through a shell timestamp tool -- piped Python output typically block-
buffers rather than flushing per line, which shows up on camera as
laggy, bursty updates instead of a smooth live feed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from datetime import datetime


class ZoneMonitor(Node):
    def __init__(self):
        super().__init__('zone_monitor')

        # Must match dntd_mmwave_safety_node's publisher QoS (RELIABLE +
        # TRANSIENT_LOCAL, depth 1) -- a default subscription here would
        # silently never receive anything, same issue we hit with the
        # arm_controller_node zone subscription earlier today.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(String, '/dntd/safety_zone', self._on_zone, qos)

    def _on_zone(self, msg: String):
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # trim micro -> milli
        print(f"{ts}  {msg.data}", flush=True)


def main():
    rclpy.init()
    node = ZoneMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
