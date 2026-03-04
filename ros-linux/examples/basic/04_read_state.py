"""
04_read_state.py — Subscribe to FR5 /joint_states topic.

Prints joint names and positions (in degrees) every second.

Start sim first:
    ros2 launch fr5_pybullet_sim fr5_sim.launch.py

Then run:
    python 04_read_state.py
Press Ctrl-C to stop.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStateReader(Node):
    """Subscribes to /joint_states and prints positions at 1 Hz."""

    def __init__(self):
        super().__init__('fr5_joint_state_reader')
        self._latest = None
        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._callback,
            10,
        )
        self._timer = self.create_timer(1.0, self._print_state)
        self.get_logger().info("Subscribed to /joint_states. Waiting for messages...")

    def _callback(self, msg):
        self._latest = msg

    def _print_state(self):
        if self._latest is None:
            self.get_logger().warn("No joint state received yet")
            return

        msg = self._latest
        print(f"\n[{msg.header.stamp.sec}.{msg.header.stamp.nanosec // 1_000_000:03d}]")
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            print(f"  {name:30s} pos: {math.degrees(pos):8.2f} deg  vel: {math.degrees(vel):7.3f} deg/s")


def main():
    rclpy.init()
    node = JointStateReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
