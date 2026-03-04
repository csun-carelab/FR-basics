"""
moveit2_demo.py — Basic MoveIt2 Python API usage with FR5.

Uses the moveit_py interface (ROS2 Humble+) to plan and execute
motions to named targets and joint goals.

Start MoveIt2 first:
    ros2 launch fairino5_v6_moveit2_config demo.launch.py

Then run (in a sourced workspace):
    python moveit2_demo.py
"""

import rclpy
from rclpy.node import Node
import math
import time
import sys


def run_with_moveit_py():
    """Run demo using moveit_py (ROS2 Humble+ with moveit_py installed)."""
    try:
        from moveit.core.robot_state import RobotState
        from moveit.planning import MoveItPy
    except ImportError:
        return False

    rclpy.init()
    moveit = MoveItPy(node_name="fr5_moveit2_demo")
    robot = moveit.get_robot()
    arm = moveit.get_planning_component("fr5_arm")

    print("MoveIt2 demo using moveit_py")
    print(f"Known named targets: {arm.get_named_targets()}")

    # Move to 'home' named target
    print("\nMoving to 'home'...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    plan_result = arm.plan()

    if plan_result:
        print("Plan succeeded. Executing...")
        moveit.execute(plan_result.trajectory, controllers=[])
        time.sleep(2.0)
    else:
        print("Planning to 'home' failed")

    # Move to a specific joint goal
    goal_deg = [0.0, -45.0, 90.0, -90.0, -90.0, 0.0]
    goal_rad = [math.radians(d) for d in goal_deg]
    print(f"\nMoving to joint goal: {goal_deg} deg")

    arm.set_start_state_to_current_state()
    robot_state = RobotState(robot.get_robot_model())
    robot_state.set_joint_group_positions("fr5_arm", goal_rad)
    arm.set_goal_state(robot_state=robot_state)

    plan_result = arm.plan()
    if plan_result:
        print("Plan succeeded. Executing...")
        moveit.execute(plan_result.trajectory, controllers=[])
        time.sleep(3.0)
    else:
        print("Planning to joint goal failed")

    print("\nDemo complete.")
    rclpy.shutdown()
    return True


def run_with_action_client():
    """Fallback: use MoveGroup action client directly."""
    from rclpy.action import ActionClient
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        MotionPlanRequest, WorkspaceParameters, Constraints,
        JointConstraint, RobotState
    )
    from sensor_msgs.msg import JointState
    import math

    class MoveGroupClient(Node):
        def __init__(self):
            super().__init__('fr5_move_group_client')
            self._ac = ActionClient(self, MoveGroup, '/move_action')
            self.get_logger().info("Waiting for MoveGroup action server...")

        def move_to_joints(self, joint_names, joint_values_rad, timeout=10.0):
            """Send a joint goal to MoveGroup."""
            if not self._ac.wait_for_server(timeout_sec=timeout):
                self.get_logger().error("MoveGroup action server not available")
                return False

            goal = MoveGroup.Goal()
            req = MotionPlanRequest()
            req.group_name = "fr5_arm"
            req.num_planning_attempts = 5
            req.allowed_planning_time = 5.0

            # Joint constraints
            constraints = Constraints()
            for name, value in zip(joint_names, joint_values_rad):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = value
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

            req.goal_constraints.append(constraints)
            goal.request = req

            self.get_logger().info(f"Sending joint goal...")
            future = self._ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

            if future.result() is None or not future.result().accepted:
                self.get_logger().error("Goal rejected")
                return False

            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
            result = result_future.result()

            if result and result.result.error_code.val == 1:
                self.get_logger().info("Motion executed successfully")
                return True
            else:
                code = result.result.error_code.val if result else "timeout"
                self.get_logger().error(f"Motion failed, error code: {code}")
                return False

    rclpy.init()
    node = MoveGroupClient()

    # FR5 joint names (from URDF)
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    goal_deg = [0.0, -45.0, 90.0, -90.0, -90.0, 0.0]
    goal_rad = [math.radians(d) for d in goal_deg]

    print(f"Moving to: {goal_deg} deg")
    try:
        node.move_to_joints(joint_names, goal_rad)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return True


def main():
    print("FR5 MoveIt2 Demo")
    print("=" * 40)

    # Try moveit_py first, fall back to action client
    try:
        import moveit
        success = run_with_moveit_py()
    except ImportError:
        print("moveit_py not found, using action client fallback...")
        try:
            import moveit_msgs
            run_with_action_client()
        except ImportError:
            print("ERROR: Neither moveit_py nor moveit_msgs found.")
            print("Source your ROS2 workspace: source install/setup.bash")
            sys.exit(1)


if __name__ == "__main__":
    main()
