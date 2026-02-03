#!/usr/bin/env python3
"""
Example: Smooth Motion with Minimum-Jerk Trajectories

Demonstrates the improved smooth motion capabilities using minimum-jerk
trajectory interpolation. Shows the difference between jerky and smooth motion.
"""

import sys
import os
import time

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python_sdk'))

from fr5_robot_interface import FR5RobotInterface


def main():
    print("=" * 70)
    print("FR5 Smooth Motion Example")
    print("=" * 70)
    print("\nThis example demonstrates smooth motion using minimum-jerk trajectories.")
    print("The robot will move through several positions with natural acceleration.\n")

    # Create robot interface in simulation mode
    with FR5RobotInterface(mode='simulation', gui=True) as robot:
        print("✓ Robot connected in simulation mode\n")

        # Define a sequence of positions
        positions = [
            {
                'name': 'Home',
                'joints': [0, 0, 0, 0, 0, 0],
                'description': 'Starting home position'
            },
            {
                'name': 'Position 1',
                'joints': [30, -20, 30, -45, 0, 30],
                'description': 'Move to first waypoint'
            },
            {
                'name': 'Position 2',
                'joints': [-30, -20, 30, -45, 0, -30],
                'description': 'Move to second waypoint'
            },
            {
                'name': 'Position 3',
                'joints': [0, -40, 60, -90, 0, 0],
                'description': 'Move to third waypoint'
            },
            {
                'name': 'Home',
                'joints': [0, 0, 0, 0, 0, 0],
                'description': 'Return to home'
            },
        ]

        # Execute motion sequence
        for i, pos in enumerate(positions, 1):
            print(f"{i}. {pos['name']}: {pos['description']}")
            print(f"   Target: {pos['joints']}")

            # Check workspace boundaries
            valid, msg = robot.check_workspace_limits(pos['joints'])
            if not valid:
                print(f"   ⚠ Warning: {msg}")
                continue

            # Execute smooth motion
            start_time = time.time()
            result = robot.move_j(pos['joints'], velocity=30)
            elapsed = time.time() - start_time

            if result == 0:
                print(f"   ✓ Motion completed in {elapsed:.2f}s")
            else:
                print(f"   ✗ Motion failed")

            # Get current state
            state = robot.get_state()
            print(f"   Current position: {[f'{j:.1f}' for j in state['joint_positions']]}\n")

            time.sleep(0.5)

        print("\n" + "=" * 70)
        print("Smooth Motion Features:")
        print("  ✓ Minimum-jerk trajectory interpolation")
        print("  ✓ Natural acceleration profiles")
        print("  ✓ Zero velocity at start and end")
        print("  ✓ Tuned PID gains (pos: 0.3, vel: 0.8)")
        print("  ✓ Collision detection with RRT-Connect")
        print("=" * 70)


if __name__ == '__main__':
    main()
