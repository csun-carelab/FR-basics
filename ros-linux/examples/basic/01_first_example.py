"""
01_first_example.py — FR5 ROS2 minimal demo.

Checks connectivity to the PyBullet simulation node by calling
the /fr5_sim/get_state Trigger service and printing the response.

Start sim first:
    ros2 launch fr5_pybullet_sim fr5_sim.launch.py

Then run:
    python 01_first_example.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class FR5BasicClient(Node):
    """Minimal ROS2 client that checks FR5 sim node connectivity."""

    def __init__(self):
        super().__init__('fr5_basic_client')
        self.cli = self.create_client(Trigger, '/fr5_sim/get_state')

    def check_connection(self, timeout_sec=5.0):
        """Wait for service and call it. Returns (success, response_message)."""
        self.get_logger().info("Waiting for /fr5_sim/get_state service...")

        if not self.cli.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                "Service not available. Is the sim node running?\n"
                "  ros2 launch fr5_pybullet_sim fr5_sim.launch.py"
            )
            return False, None

        self.get_logger().info("Service found. Sending request...")
        future = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            self.get_logger().error("Service call timed out")
            return False, None

        response = future.result()
        return response.success, response.message


def main():
    rclpy.init()
    node = FR5BasicClient()

    try:
        success, message = node.check_connection()

        if success:
            print("\nFR5 sim node is running.")
            print(f"Response: {message}")
        else:
            print("\nFailed to connect to FR5 sim node.")
            if message:
                print(f"Message: {message}")
            sys.exit(1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
