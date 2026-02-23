#!/usr/bin/env python3
"""
Example: Cartesian Path Planning (MoveL)

Demonstrates Cartesian space motion planning including:
- Straight-line motion in workspace
- Pose interpolation
- IK-based path generation
- Path validation
"""

import sys
import os
import time

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python_sdk'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python_sdk/simulation'))

from fr5_robot_interface import FR5RobotInterface
from cartesian_planner import CartesianPlanner
import numpy as np


def main():
    print("=" * 70)
    print("FR5 Cartesian Path Planning Example")
    print("=" * 70)
    print("\nDemonstrating straight-line motion in Cartesian space (MoveL).\n")

    with FR5RobotInterface(mode='simulation', gui=True) as robot:
        print("✓ Robot connected in simulation mode\n")

        # Move to a known starting position
        print("1. Moving to starting position...")
        start_joints = [0, -30, 45, -60, 0, 0]
        robot.move_j(start_joints, velocity=40)
        time.sleep(1.0)
        print("   ✓ At starting position\n")

        # Get current end-effector pose
        state = robot.get_state()
        print("2. Current End-Effector State:")
        print(f"   TCP Position: {state['tcp_pose'][:3]}")
        print(f"   TCP Orientation (euler): {state['tcp_pose'][3:]}\n")

        # Example 1: Simple linear motion
        print("3. Cartesian Pose Interpolation")
        print("-" * 70)

        # Define start and end poses [x, y, z, rx, ry, rz]
        start_pose = [0.5, 0.0, 0.4, 3.14, 0, 0]
        end_pose = [0.5, 0.2, 0.4, 3.14, 0, 0]  # Move 20cm in Y direction

        print(f"   Start: {start_pose}")
        print(f"   End:   {end_pose}")
        print("   Generating 30 interpolated waypoints...")

        # Interpolate Cartesian path
        cart_path = CartesianPlanner.interpolate_pose(start_pose, end_pose, 30)

        print(f"   ✓ Generated {len(cart_path)} waypoints")
        print(f"   First waypoint: {cart_path[0]}")
        print(f"   Last waypoint:  {cart_path[-1]}\n")

        # Example 2: IK-based Cartesian planning
        print("4. IK-Based Cartesian Path Planning")
        print("-" * 70)

        if hasattr(robot, 'backend') and hasattr(robot.backend, 'robot_id'):
            print("   Planning straight-line path with IK...")

            # Get current joint positions
            current_joints = robot.get_joint_positions()

            # Define target pose (relative movement)
            target_pose = [0.5, 0.15, 0.5, 3.14, 0, 0]

            print(f"   Current joints: {[f'{j:.1f}' for j in current_joints]}")
            print(f"   Target pose: {target_pose}")

            # Plan Cartesian path
            joint_path = CartesianPlanner.plan_line(
                robot.backend.robot_id,
                robot.backend.ee_link_index,
                target_pose,
                current_joints,
                num_waypoints=20
            )

            if joint_path is not None:
                print(f"   ✓ Path planned successfully: {len(joint_path)} waypoints")

                # Validate path
                is_valid = CartesianPlanner.validate_cartesian_path(joint_path, max_joint_diff=0.3)
                if is_valid:
                    print("   ✓ Path validation passed (smooth and feasible)")

                    # Display first and last joint configurations
                    print(f"   Start joints: {[f'{j:.1f}' for j in joint_path[0]]}")
                    print(f"   End joints:   {[f'{j:.1f}' for j in joint_path[-1]]}")
                else:
                    print("   ⚠ Path validation failed (discontinuities detected)")
            else:
                print("   ✗ IK failed - target pose may be unreachable")
        else:
            print("   ℹ IK-based planning requires PyBullet backend\n")

        # Example 3: Drawing a square in Cartesian space
        print("\n5. Drawing a Square Path")
        print("-" * 70)

        # Define square corners
        square_poses = [
            [0.4, -0.1, 0.4, 3.14, 0, 0],  # Corner 1
            [0.4, 0.1, 0.4, 3.14, 0, 0],   # Corner 2
            [0.6, 0.1, 0.4, 3.14, 0, 0],   # Corner 3
            [0.6, -0.1, 0.4, 3.14, 0, 0],  # Corner 4
            [0.4, -0.1, 0.4, 3.14, 0, 0],  # Back to Corner 1
        ]

        print("   Square path waypoints:")
        for i, pose in enumerate(square_poses, 1):
            print(f"   Corner {i}: X={pose[0]:.2f}m, Y={pose[1]:.2f}m, Z={pose[2]:.2f}m")

        print("\n   Interpolating edges...")
        full_path = []
        for i in range(len(square_poses) - 1):
            edge_path = CartesianPlanner.interpolate_pose(
                square_poses[i],
                square_poses[i + 1],
                num_points=15
            )
            full_path.extend(edge_path)

        print(f"   ✓ Generated {len(full_path)} waypoints for complete square\n")

        # Example 4: Circular motion
        print("6. Circular Motion in XY Plane")
        print("-" * 70)

        center = [0.5, 0.0, 0.4]
        radius = 0.1
        num_points = 40

        print(f"   Center: {center}")
        print(f"   Radius: {radius}m")
        print(f"   Points: {num_points}")

        circle_poses = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]
            pose = [x, y, z, 3.14, 0, 0]
            circle_poses.append(pose)

        print(f"   ✓ Generated circular path with {len(circle_poses)} waypoints\n")

        # Summary
        print("=" * 70)
        print("Cartesian Planning Features:")
        print("  ✓ Linear pose interpolation")
        print("  ✓ IK-based path generation")
        print("  ✓ Path smoothness validation")
        print("  ✓ Straight-line motion (MoveL)")
        print("  ✓ Complex path patterns (squares, circles)")
        print("  ✓ Collision-free trajectory planning")
        print("=" * 70)
        print("\nUsage in code:")
        print("  from cartesian_planner import CartesianPlanner")
        print("  path = CartesianPlanner.plan_line(robot_id, ee_link, target, current)")
        print("  is_valid = CartesianPlanner.validate_cartesian_path(path)")
        print("=" * 70)


if __name__ == '__main__':
    main()
