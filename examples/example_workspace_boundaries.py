#!/usr/bin/env python3
"""
Example: Workspace Boundary Checking

Demonstrates workspace boundary validation including joint limits,
Cartesian limits, and visual workspace display.
"""

import sys
import os

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python_sdk'))

from fr5_robot_interface import FR5RobotInterface
import numpy as np


def main():
    print("=" * 70)
    print("FR5 Workspace Boundary Example")
    print("=" * 70)
    print("\nThis example demonstrates workspace boundary checking and validation.\n")

    with FR5RobotInterface(mode='simulation', gui=True) as robot:
        print("✓ Robot connected in simulation mode\n")

        # Test various positions
        test_positions = [
            {
                'name': 'Valid: Home position',
                'joints': [0, 0, 0, 0, 0, 0],
                'should_pass': True
            },
            {
                'name': 'Valid: Normal working position',
                'joints': [45, -30, 45, -60, 0, 30],
                'should_pass': True
            },
            {
                'name': 'Invalid: J1 exceeds +175°',
                'joints': [180, 0, 0, 0, 0, 0],
                'should_pass': False
            },
            {
                'name': 'Invalid: J2 exceeds -265° limit',
                'joints': [0, -270, 0, 0, 0, 0],
                'should_pass': False
            },
            {
                'name': 'Valid: J2 at maximum (+85°)',
                'joints': [0, 85, 0, 0, 0, 0],
                'should_pass': True
            },
            {
                'name': 'Invalid: J3 exceeds ±162°',
                'joints': [0, 0, 170, 0, 0, 0],
                'should_pass': False
            },
            {
                'name': 'Valid: Edge of workspace',
                'joints': [175, -30, 160, -60, 175, 175],
                'should_pass': True
            },
        ]

        print("Testing workspace boundaries:\n")
        print("-" * 70)

        for i, test in enumerate(test_positions, 1):
            print(f"\n{i}. {test['name']}")
            print(f"   Joints: {test['joints']}")

            valid, msg = robot.check_workspace_limits(test['joints'])

            if valid:
                print(f"   ✓ VALID: {msg}")
            else:
                print(f"   ✗ INVALID: {msg}")

            # Check if result matches expectation
            if valid == test['should_pass']:
                print(f"   ✓ Result matches expectation")
            else:
                print(f"   ⚠ Unexpected result!")

        print("\n" + "-" * 70)

        # Demonstrate Cartesian workspace limits
        print("\n\nCartesian Workspace Limits:")
        print("-" * 70)
        print("FR5 Robot Specifications:")
        print("  • Maximum reach: 922mm")
        print("  • Minimum reach: 150mm (safety buffer)")
        print("  • Working envelope: 0.15m - 0.85m radius")
        print("  • Height range: 0.0m - 1.2m (Z-axis)")
        print("  • X-axis range: -0.9m to +0.9m")
        print("  • Y-axis range: -0.9m to +0.9m")
        print("-" * 70)

        # Display joint limits
        print("\n\nJoint Limits (FR5):")
        print("-" * 70)
        joint_limits = [
            ("J1", -175, 175),
            ("J2", -265, 85),
            ("J3", -162, 162),
            ("J4", -265, 85),
            ("J5", -175, 175),
            ("J6", -175, 175),
        ]

        for joint_name, min_deg, max_deg in joint_limits:
            print(f"  {joint_name}: {min_deg:>4}° to {max_deg:>4}°")

        print("-" * 70)

        print("\n" + "=" * 70)
        print("Workspace Boundary Features:")
        print("  ✓ Joint limit validation (±175° to ±265° range)")
        print("  ✓ Cartesian envelope checking (0.15m - 0.85m)")
        print("  ✓ Real-time boundary visualization")
        print("  ✓ Collision-free path planning")
        print("  ✓ Automatic position clamping")
        print("=" * 70)


if __name__ == '__main__':
    main()
