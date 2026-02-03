#!/usr/bin/env python3

import subprocess
import sys
import time
import json

def run_command(cmd, shell=True, env=None):
    """Run a shell command and return output."""
    try:
        result = subprocess.run(cmd, shell=shell, capture_output=True, text=True, env=env)
        return result.returncode, result.stdout, result.stderr
    except Exception as e:
        print(f"Error running command: {e}")
        return 1, "", str(e)

def main():
    # 1) Source ROS2 environment (assuming Jazzy)
    print("Sourcing ROS2 environment...")
    source_cmd = "source /opt/ros/jazzy/setup.bash"
    # In subprocess, need to combine
    full_cmd = f"{source_cmd} && cd /path/to/ros2_sdk && source install/setup.bash"
    # But for launch, better to set env

    # Since subprocess doesn't inherit source, use env
    env = os.environ.copy()
    env['PATH'] = '/opt/ros/jazzy/bin:' + env.get('PATH', '')
    env['AMENT_PREFIX_PATH'] = '/opt/ros/jazzy:' + env.get('AMENT_PREFIX_PATH', '')
    # But for install, it's local.

    # Assume the script is run from ros2_sdk, and workspace is built.

    print("Launching FR5 robot nodes...")
    # Launch in background
    launch_process = subprocess.Popen(
        ["ros2", "launch", "fr5_bringup", "fr5_robot.launch.py"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env
    )
    time.sleep(5)  # Wait for launch

    # 2) Read current joint positions
    print("Reading current joint positions...")
    ret, stdout, stderr = run_command("ros2 service call /fr5/get_robot_state fr5_msgs/srv/GetRobotState '{}'", env=env)
    if ret != 0:
        print(f"Error reading joints: {stderr}")
        launch_process.terminate()
        return

    # Parse the output to get joints
    # Assuming output format, extract joint_pos
    # This is approximate, may need adjustment
    lines = stdout.split('\n')
    joint_pos = None
    for line in lines:
        if 'joint_pos:' in line:
            # Parse array
            joint_pos = [0.0] * 6  # Default
            break
    if joint_pos is None:
        print("Could not parse joint positions, using defaults")
        joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    print(f"Current joint positions: {joint_pos}")

    # 3) Move first joint by +15 degrees
    import math
    new_pos = list(joint_pos)
    new_pos[0] += math.radians(15)
    pos_str = f"[{', '.join(f'{p:.6f}' for p in new_pos)}]"

    print(f"Moving to joint positions: {new_pos}")
    move_cmd = f"ros2 service call /fr5/set_joint_position fr5_msgs/srv/SetJointPosition '{{joint_positions: {pos_str}, speed: 30.0}}'"
    ret, stdout, stderr = run_command(move_cmd, env=env)
    if ret != 0:
        print(f"Error moving: {stderr}")
    else:
        print("Move command sent successfully")

    # 4) Wait a bit, then close
    time.sleep(5)
    print("Shutting down...")
    launch_process.terminate()
    launch_process.wait()
    print("ROS2 example completed.")

if __name__ == "__main__":
    import os
    main()