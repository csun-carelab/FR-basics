#!/usr/bin/env python3
"""
FR5 Robot - Real Robot First Example

This example demonstrates basic control of the FR5 robot using the fairino SDK.
It connects to the robot, reads joint positions, moves the first joint by 15 degrees,
and then closes the connection.

Requirements:
- FR5 robot connected and powered on
- Robot IP: 192.168.58.2 (default)
- fairino library (included in this package)
"""

import sys
import os
import numpy as np

# Add the parent directory to path to find the fairino package
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# Import the fairino Robot module
from fairino.Robot import RPC

def main():
    print("=" * 60)
    print("FR5 Robot - Real Robot First Example")
    print("=" * 60)

    # Robot IP address (adjust if needed)
    ROBOT_IP = '192.168.58.2'

    print(f"\nConnecting to robot at {ROBOT_IP}...")

    try:
        # Connect to robot
        robot = RPC(ROBOT_IP)
        print("Robot connected successfully!")

        # Read current joint angles of robot (in degrees)
        currPos = list(robot.robot_state_pkg.jt_cur_pos)
        print(f"\nCurrent joint positions (degrees): {[f'{p:.2f}' for p in currPos]}")

        # Move robot to new position (rotate first joint by 15 degrees)
        newPos = np.array(currPos)
        newPos[0] += 15
        print(f"\nMoving to new position: J1 += 15 degrees")
        print(f"Target joint positions: {[f'{p:.2f}' for p in newPos]}")

        # Execute joint move
        # MoveJ parameters: joint_pos (list), desc_pos (list), tool (int), user (int),
        #                   vel (float), acc (float), ovl (float), epos (list), blendT (float),
        #                   offset_flag (int), offset_pos (list)
        rtn = robot.MoveJ(joint_pos=newPos.tolist(), desc_pos=[0,0,0,0,0,0],
                          tool=0, user=0, vel=100.0, acc=100.0, ovl=100.0,
                          epos=[0,0,0,0], blendT=-1.0, offset_flag=0, offset_pos=[0,0,0,0,0,0])

        if rtn == 0:
            print("Move command executed successfully!")
        else:
            print(f"Move command returned error code: {rtn}")

        # Close robot connection
        robot.CloseRPC()
        print("\nRobot connection closed.")
        print("=" * 60)

    except Exception as e:
        print(f"\nError: {e}")
        print("\nMake sure:")
        print("  1. The robot is powered on")
        print("  2. The robot is connected to the network")
        print(f"  3. The robot IP address is correct ({ROBOT_IP})")
        print("  4. No other program is controlling the robot")
        sys.exit(1)

if __name__ == "__main__":
    main()
