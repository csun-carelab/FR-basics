"""
01_first_example.py — FR5 minimal connection and motion demo.

Sim:  python 01_first_example.py
Real: python 01_first_example.py --real [--ip 192.168.58.2]

Demonstrates:
- Connecting to simulation (PyBullet) or real robot (Fairino SDK)
- Moving to home position
- Reading and printing joint state
"""

import argparse
import sys
import os
import math
import time

# Add python-sdk to path
SDK_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python-sdk')
sys.path.insert(0, os.path.abspath(SDK_DIR))

HOME_DEG = [0.0, -70.0, 90.0, -110.0, -90.0, 30.0]


def run_sim():
    """Run minimal demo in PyBullet simulation."""
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print("ERROR: pybullet not installed. Run: pip install pybullet")
        sys.exit(1)

    urdf_path = os.path.join(SDK_DIR, 'urdf', 'fairino5_v6.urdf')
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF not found at {urdf_path}")
        sys.exit(1)

    # Patch package:// mesh paths to absolute paths for PyBullet
    import re, tempfile
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    with open(urdf_path, "r", encoding="utf-8") as fh:
        urdf_content = fh.read()
    mesh_dir = os.path.join(urdf_dir, "meshes").replace("\\", "/")
    urdf_content = re.sub(
        r"package://fairino_description/meshes/fairino5_v6",
        mesh_dir,
        urdf_content,
    )
    fd, patched_urdf = tempfile.mkstemp(suffix=".urdf", text=True)
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(urdf_content)

    print("Starting PyBullet simulation...")
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(1.8, 45, -30, [0.3, 0, 0.4])
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(patched_urdf, useFixedBase=True)

    # Find arm joints
    arm_joints = []
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        if info[2] == p.JOINT_REVOLUTE:
            arm_joints.append(j)

    print(f"Robot loaded. Found {len(arm_joints)} revolute joints.")

    # Move to home position
    print(f"Moving to home: {HOME_DEG} deg")
    for idx, jid in enumerate(arm_joints[:6]):
        p.resetJointState(robot_id, jid, math.radians(HOME_DEG[idx]))

    # Read back and print joint positions
    print("\nJoint positions (deg):")
    for idx, jid in enumerate(arm_joints[:6]):
        pos_rad = p.getJointState(robot_id, jid)[0]
        print(f"  J{idx+1}: {math.degrees(pos_rad):.2f} deg")

    print("\nSimulation running. Close the window or press Ctrl-C to exit.")
    try:
        while p.isConnected(client):
            p.stepSimulation()
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        if os.path.exists(patched_urdf):
            os.unlink(patched_urdf)


def run_real(ip):
    """Run minimal demo on real FR5 robot."""
    try:
        import Robot
    except ImportError:
        print("ERROR: fairino SDK not installed.")
        sys.exit(1)

    print(f"Connecting to FR5 at {ip} ...")
    robot = Robot.RPC(ip)
    print("Connected.")

    # Read current joint state
    ret = robot.GetActualJointPosDegree()
    print("\nCurrent joint positions (deg):")
    joints = ret[1] if isinstance(ret, (list, tuple)) and len(ret) >= 2 else ret
    for i, angle in enumerate(joints[:6]):
        print(f"  J{i+1}: {float(angle):.2f} deg")

    print(f"\nMoving to home position: {HOME_DEG}")
    robot.MoveJ(joint_pos=HOME_DEG, tool=0, user=0, vel=20)
    for _ in range(50):
        time.sleep(0.1)
        ret = robot.GetRobotMotionDone()
        if isinstance(ret, (list, tuple)) and ret[0] == 0 and ret[1] == 1:
            break

    ret = robot.GetActualJointPosDegree()
    joints = ret[1] if isinstance(ret, (list, tuple)) and len(ret) >= 2 else ret
    print("\nPosition after move (deg):")
    for i, angle in enumerate(joints[:6]):
        print(f"  J{i+1}: {float(angle):.2f} deg")

    print("\nDone.")
    robot.CloseRPC()


def main():
    parser = argparse.ArgumentParser(description="FR5 first example")
    parser.add_argument("--real", action="store_true", help="Connect to real robot")
    parser.add_argument("--ip", default="192.168.58.2", help="Robot IP (real only)")
    args = parser.parse_args()

    if args.real:
        run_real(args.ip)
    else:
        run_sim()


if __name__ == "__main__":
    main()
