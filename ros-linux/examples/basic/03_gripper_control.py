"""
03_gripper_control.py — AG-95 gripper control via ROS2 services.

Demonstrates opening and closing the gripper using the FR5 sim node services.

Start sim first (with AG-95 gripper URDF):
    ros2 launch fr5_pybullet_sim fr5_sim.launch.py urdf_name:=fairino5_v6_with_ag95.urdf

Then run:
    python 03_gripper_control.py [--action open|close|cycle]
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import argparse
import time
import sys


class GripperClient(Node):
    """ROS2 client for FR5 gripper control."""

    def __init__(self):
        super().__init__('fr5_gripper_client')
        self.open_cli = self.create_client(Trigger, '/fr5_sim/open_gripper')
        self.close_cli = self.create_client(Trigger, '/fr5_sim/close_gripper')

    def _call(self, client, name, timeout_sec=5.0):
        """Wait for service and call it."""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"Service {name} not available")
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            self.get_logger().error(f"Service {name} timed out")
            return False

        if future.result().success:
            self.get_logger().info(f"{name}: OK — {future.result().message}")
        else:
            self.get_logger().warn(f"{name}: {future.result().message}")

        return future.result().success

    def open(self):
        """Open gripper fully."""
        self.get_logger().info("Opening gripper...")
        return self._call(self.open_cli, '/fr5_sim/open_gripper')

    def close(self):
        """Close gripper."""
        self.get_logger().info("Closing gripper...")
        return self._call(self.close_cli, '/fr5_sim/close_gripper')


def main():
    parser = argparse.ArgumentParser(description="FR5 gripper control via ROS2")
    parser.add_argument("--action", choices=["open", "close", "cycle"], default="cycle",
                        help="Gripper action (default: cycle open->close->open)")
    args = parser.parse_args()

    rclpy.init()
    node = GripperClient()

    try:
        if args.action == "open":
            node.open()
        elif args.action == "close":
            node.close()
        else:  # cycle
            print("Gripper cycle: open -> wait 1s -> close -> wait 1s -> open")
            node.open()
            time.sleep(1.0)
            node.close()
            time.sleep(1.0)
            node.open()
            print("Cycle complete.")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
