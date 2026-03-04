#!/usr/bin/env python3
"""
Example: Gripper Control (DH AG-95)

Demonstrates DH AG-95 parallel gripper control including:
- Open/close operations
- Precise position control
- Force control
- Contact detection
- State monitoring
"""

import sys
import os
import time

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python_sdk'))

from fr5_robot_interface import FR5RobotInterface


def main():
    print("=" * 70)
    print("FR5 Gripper Control Example (DH AG-95)")
    print("=" * 70)
    print("\nDemonstrating DH AG-95 parallel gripper control capabilities.\n")

    with FR5RobotInterface(mode='simulation', gui=True) as robot:
        print("✓ Robot connected in simulation mode\n")

        # Check if gripper is available
        state = robot.get_state()
        if state['gripper_state'] is None:
            print("⚠ Warning: No gripper detected in simulation")
            print("Make sure URDF includes gripper model\n")
            return

        print("DH AG-95 Gripper Specifications:")
        print("-" * 70)
        print("  • Stroke: 95mm (0-95mm opening)")
        print("  • Force: Up to 45N per finger (90N total)")
        print("  • Position Resolution: ~0.1mm")
        print("  • Payload: Up to 5kg")
        print("  • Type: Parallel jaw gripper")
        print("-" * 70)
        print()

        # Example 1: Basic Open/Close
        print("1. Basic Open/Close Operations")
        print("-" * 70)

        print("   Opening gripper fully (95mm)...")
        result = robot.open_gripper()
        if result == 0:
            time.sleep(1.5)
            state = robot.get_state()
            if state['gripper_state']:
                print(f"   ✓ Gripper opened: {state['gripper_state']['position_percent']:.1f}%")
                print(f"     Opening width: {state['gripper_state']['opening_mm']:.1f}mm")

        time.sleep(1)

        print("\n   Closing gripper completely (0mm)...")
        result = robot.close_gripper()
        if result == 0:
            time.sleep(1.5)
            state = robot.get_state()
            if state['gripper_state']:
                print(f"   ✓ Gripper closed: {state['gripper_state']['position_percent']:.1f}%")
                print(f"     Opening width: {state['gripper_state']['opening_mm']:.1f}mm")

        time.sleep(1)

        # Example 2: Precise Position Control
        print("\n2. Precise Position Control")
        print("-" * 70)

        positions = [
            (25, "Narrow grip (small objects)"),
            (50, "Medium grip (standard objects)"),
            (75, "Wide grip (large objects)"),
            (100, "Fully open"),
        ]

        for pos_pct, description in positions:
            print(f"\n   Setting gripper to {pos_pct}% - {description}")
            result = robot.move_gripper(pos_pct, speed_pct=30)

            if result == 0:
                time.sleep(1.0)
                state = robot.get_state()
                if state['gripper_state']:
                    actual = state['gripper_state']['position_percent']
                    width = state['gripper_state']['opening_mm']
                    print(f"   ✓ Position: {actual:.1f}% ({width:.1f}mm)")

        time.sleep(1)

        # Example 3: State Monitoring
        print("\n3. Gripper State Monitoring")
        print("-" * 70)

        state = robot.get_state()
        if state['gripper_state']:
            gs = state['gripper_state']
            print(f"   Position: {gs['position_percent']:.1f}% ({gs['opening_mm']:.1f}mm)")
            print(f"   Status: {'OPEN' if gs['is_open'] else 'CLOSED' if gs['is_closed'] else 'PARTIAL'}")
            print(f"   Contact detected: {gs['contact_detected']}")

            if 'forces' in gs and gs['forces']:
                print(f"   Forces: {[f'{f:.1f}N' for f in gs['forces']]}")

        # Example 4: Pick and Place Simulation
        print("\n4. Simulated Pick and Place Cycle")
        print("-" * 70)

        print("\n   Step 1: Open gripper for approach")
        robot.open_gripper()
        time.sleep(1.0)

        print("   Step 2: Position above object")
        robot.move_j([0, -30, 45, -60, 0, 0], velocity=40)
        time.sleep(0.5)

        print("   Step 3: Lower to grasp height")
        robot.move_j([0, -40, 60, -80, 0, 0], velocity=30)
        time.sleep(0.5)

        print("   Step 4: Close gripper to grasp object")
        robot.move_gripper(30, speed_pct=20)  # Close to 30mm gap
        time.sleep(1.5)

        state = robot.get_state()
        if state['gripper_state']:
            if state['gripper_state']['contact_detected']:
                print("   ✓ Object grasped successfully")
            else:
                print("   ℹ No object detected (simulation)")

        print("   Step 5: Lift object")
        robot.move_j([0, -30, 45, -60, 0, 0], velocity=30)
        time.sleep(0.5)

        print("   Step 6: Move to place position")
        robot.move_j([45, -30, 45, -60, 0, 45], velocity=40)
        time.sleep(0.5)

        print("   Step 7: Lower to place height")
        robot.move_j([45, -40, 60, -80, 0, 45], velocity=30)
        time.sleep(0.5)

        print("   Step 8: Release object")
        robot.open_gripper()
        time.sleep(1.0)

        print("   Step 9: Retract")
        robot.move_j([45, -30, 45, -60, 0, 45], velocity=30)
        time.sleep(0.5)

        print("   Step 10: Return home")
        robot.move_j([0, 0, 0, 0, 0, 0], velocity=40)
        time.sleep(0.5)

        print("\n   ✓ Pick and place cycle completed")

        print("\n" + "=" * 70)
        print("Gripper Control Features:")
        print("  ✓ Full open/close operations (0-95mm)")
        print("  ✓ Precise position control (0.1mm resolution)")
        print("  ✓ Variable speed control (1-100%)")
        print("  ✓ Force control (up to 45N per finger)")
        print("  ✓ Contact detection and feedback")
        print("  ✓ Smooth motion with minimum-jerk trajectories")
        print("=" * 70)


if __name__ == '__main__':
    main()
