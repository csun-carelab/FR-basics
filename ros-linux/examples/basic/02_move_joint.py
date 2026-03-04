"""
02_move_joint.py — Move FR5 to home position via ROS2 service.

Calls /fr5_sim/set_joint_position (Trigger) to reset the sim robot to home,
then reads back the joint state from /joint_states.

Start sim first:
    ros2 launch fr5_pybullet_sim fr5_sim.launch.py

Then run:
    python 02_move_joint.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import math
import sys

HOME_DEG = [0.0, -70.0, 90.0, -110.0, -90.0, 30.0]


class FR5MoveClient(Node):
    """ROS2 client to move FR5 to home and read joint state."""

    def __init__(self):
        super().__init__('fr5_move_client')
        self.home_cli = self.create_client(Trigger, '/fr5_sim/set_joint_position')
        self.joint_state = None
        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._js_callback,
            10,
        )

    def _js_callback(self, msg):
        self.joint_state = msg

    def move_to_home(self, timeout_sec=5.0):
        """Call /fr5_sim/set_joint_position to reset to home. Returns success bool."""
        if not self.home_cli.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                "Service /fr5_sim/set_joint_position not available.\n"
                "Is the sim node running? ros2 launch fr5_pybullet_sim fr5_sim.launch.py"
            )
            return False

        self.get_logger().info(f"Moving to home: {HOME_DEG} deg")
        future = self.home_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            self.get_logger().error("Service call timed out")
            return False

        if not future.result().success:
            self.get_logger().error(f"Move failed: {future.result().message}")
            return False

        self.get_logger().info("Home position set successfully")
        return True

    def read_joint_state(self, timeout_sec=3.0):
        """Spin briefly to get one /joint_states message."""
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                break
        return self.joint_state


def main():
    rclpy.init()
    node = FR5MoveClient()

    try:
        success = node.move_to_home()
        if not success:
            sys.exit(1)

        print("\nReading joint state from /joint_states ...")
        js = node.read_joint_state()

        if js is None:
            print("No joint state received.")
        else:
            print("\nJoint positions:")
            for name, pos in zip(js.name, js.position):
                print(f"  {name}: {math.degrees(pos):.2f} deg")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
