"""
fake_joint_states.py
DNTD Dynamics — Stick Test Helper

Publishes static zero joint states so the safety node doesn't raise
a joint_states fault during stick testing (before the arm exists).

Usage:
  python3 fake_joint_states.py                          # default 6 joints
  python3 fake_joint_states.py --joints joint1 joint2   # custom names
  python3 fake_joint_states.py --hz 50                  # publish rate

With ego-motion compensation effectively zeroed (all joints stationary),
any doppler velocity in the radar return IS the real-world velocity.
That's correct for stick testing — the "arm" isn't moving.

When you move the stick manually, the sensor is moving but joint_states
says it isn't — so compensated velocity will include your hand motion.
That's expected and fine for threshold tuning. It just means your readings
are in sensor-frame, not world-frame, which is fine for a handheld test.
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeJointStatePublisher(Node):

    def __init__(self, joint_names: list[str], hz: float):
        super().__init__('fake_joint_states')

        self._joint_names = joint_names
        self._n = len(joint_names)

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(
            f"Publishing fake joint states\n"
            f"  joints : {joint_names}\n"
            f"  rate   : {hz} Hz\n"
            f"  Note   : all positions=0, velocities=0 — ego-motion compensation disabled\n"
            f"  This is correct for stick testing. Stop with Ctrl+C."
        )

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self._joint_names
        msg.position     = [0.0] * self._n
        msg.velocity     = [0.0] * self._n
        msg.effort       = [0.0] * self._n
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Fake joint state publisher for stick testing")
    parser.add_argument(
        "--joints", nargs="+",
        default=["joint1","joint2","joint3","joint4","joint5","joint6"],
        help="Joint names to publish (must match safety node config)",
    )
    parser.add_argument(
        "--hz", type=float, default=50.0,
        help="Publish rate in Hz (default: 50)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = FakeJointStatePublisher(args.joints, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
