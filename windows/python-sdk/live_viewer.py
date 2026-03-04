"""
FR5 Live Viewer — mirrors real robot joints into PyBullet at ~10 Hz.

Connects to a running FR5 robot controller, reads joint positions,
and updates a PyBullet 3D visualization in real time.

Usage:
    python live_viewer.py [robot_ip]
    Default IP: 192.168.58.2

Requirements:
    pip install pybullet
    fairino Python SDK installed (from fairino.Robot import RPC)
"""

import pybullet as p
import pybullet_data
import time
import math
import os
import argparse
import re
import tempfile

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf", "fairino5_v6_with_ag95.urdf")
UPDATE_HZ = 10
STATUS_INTERVAL = 5.0   # Print status every N seconds


def _patch_urdf(urdf_path):
    """Replace package:// mesh paths with absolute paths for PyBullet."""
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    with open(urdf_path, 'r') as f:
        content = f.read()

    meshes_dir = os.path.join(urdf_dir, 'fairino5_v6')
    content = re.sub(r'package://fairino_description/meshes/fairino5_v6', meshes_dir, content)

    ag95_dir = os.path.join(urdf_dir, 'meshes', 'ag95')
    content = re.sub(r'package://ag95_meshes', ag95_dir, content)

    fd, tmp = tempfile.mkstemp(suffix='.urdf', text=True)
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return tmp


def main(robot_ip="192.168.58.2"):
    try:
        from fairino.Robot import RPC
    except ImportError:
        print("ERROR: fairino SDK not found. Install the Fairino Python SDK.")
        return

    print(f"Connecting to FR5 at {robot_ip} ...")
    robot = RPC(robot_ip)
    print("Connected.")

    # PyBullet setup
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(2.0, 45, -30, [0.3, 0, 0.5])
    p.loadURDF("plane.urdf")

    patched = _patch_urdf(URDF_PATH)
    robot_id = p.loadURDF(
        patched,
        basePosition=[0.0, 0.0, 0.0],
        baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi]),
        useFixedBase=True,
    )
    os.unlink(patched)

    # Find arm joints
    arm_joints = []
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        name = info[1].decode()
        if info[2] == p.JOINT_REVOLUTE and 'gripper' not in name.lower():
            arm_joints.append(j)

    print(f"Visualizing {len(arm_joints)} arm joints. Press Ctrl-C to exit.")
    p.addUserDebugText("FR5 Live Viewer", [0.3, 0, 1.0], textColorRGB=[0, 1, 0], textSize=1.2)

    dt = 1.0 / UPDATE_HZ
    last_status = time.time()
    frame = 0

    try:
        while True:
            ret = robot.GetActualJointPosDegree()
            # ret is [error_code, [j1, j2, j3, j4, j5, j6]] or similar
            if isinstance(ret, (list, tuple)) and len(ret) >= 2:
                joints_deg = ret[1] if isinstance(ret[1], (list, tuple)) else ret
            else:
                joints_deg = ret

            if joints_deg and len(joints_deg) >= 6:
                for idx, joint_id in enumerate(arm_joints[:6]):
                    angle_rad = math.radians(float(joints_deg[idx]))
                    p.resetJointState(robot_id, joint_id, angle_rad)

            p.stepSimulation()
            frame += 1

            now = time.time()
            if now - last_status >= STATUS_INTERVAL:
                deg_str = ', '.join(f"{float(d):.1f}" for d in joints_deg[:6]) if joints_deg else "N/A"
                print(f"[{frame}] Joints: [{deg_str}] deg")
                last_status = now

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        p.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FR5 Live Viewer — PyBullet mirror of real robot")
    parser.add_argument("ip", nargs="?", default="192.168.58.2", help="Robot IP address")
    args = parser.parse_args()
    main(args.ip)
