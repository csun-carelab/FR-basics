"""
Unified FR5 Robot Interface.

Provides one API for real hardware (Fairino SDK) and PyBullet simulation.
"""

import math
import os
import re
import sys
import tempfile
from typing import Dict, List, Optional, Tuple

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


class FR5RobotInterface:
    """Unified interface for FAIRINO FR5 robot in simulation and real modes."""

    def __init__(
        self,
        mode: str = "simulation",
        robot_ip: str = "192.168.58.2",
        gui: bool = True,
        urdf_path: Optional[str] = None,
    ):
        self.mode = mode
        self.robot_ip = robot_ip
        self.backend = None
        self.gripper_controller = None

        # Simulation-only handles
        self._p = None
        self._physics_client = None
        self._robot_id = None
        self.arm_joint_indices: List[int] = []
        self.finger_joint_indices: List[int] = []
        self._patched_urdf: Optional[str] = None

        print(f"Initializing FR5RobotInterface in {mode} mode...")

        if mode == "simulation":
            self._init_simulation(gui, urdf_path)
        elif mode == "real":
            self._init_real_robot(robot_ip)
        else:
            raise ValueError(f"Invalid mode: {mode}. Must be 'simulation' or 'real'")

        print(f"[OK] FR5RobotInterface ready in {mode} mode")

    @staticmethod
    def _patch_urdf_mesh_paths(urdf_path: str) -> str:
        """Patch package:// mesh paths to absolute paths so PyBullet can load them."""
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        with open(urdf_path, "r", encoding="utf-8") as fh:
            content = fh.read()

        fr_mesh_dir = os.path.join(urdf_dir, "meshes")
        ag95_mesh_dir = os.path.join(urdf_dir, "meshes", "ag95")
        content = re.sub(
            r"package://fairino_description/meshes/fairino5_v6",
            fr_mesh_dir.replace("\\", "/"),
            content,
        )
        content = re.sub(r"package://ag95_meshes", ag95_mesh_dir.replace("\\", "/"), content)

        fd, patched_path = tempfile.mkstemp(suffix=".urdf", text=True)
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(content)
        return patched_path

    def _init_simulation(self, gui: bool, urdf_path: Optional[str]):
        """Initialize PyBullet simulation backend."""
        try:
            import pybullet as p
            import pybullet_data
            from gripper_controller import GripperController
        except ImportError as e:
            raise ImportError(f"Failed to import simulation modules: {e}")

        self._p = p
        self._physics_client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        if urdf_path is None:
            urdf_path = os.path.join(SCRIPT_DIR, "urdf", "fairino5_v6_with_ag95.urdf")

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"Simulation URDF not found: {urdf_path}")

        self._patched_urdf = self._patch_urdf_mesh_paths(urdf_path)
        self._robot_id = p.loadURDF(
            self._patched_urdf,
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi]),
            useFixedBase=True,
        )

        self.arm_joint_indices = []
        self.finger_joint_indices = []
        for jid in range(p.getNumJoints(self._robot_id)):
            info = p.getJointInfo(self._robot_id, jid)
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            is_active = joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
            if not is_active:
                continue

            lname = joint_name.lower()
            if "gripper" in lname or "finger" in lname:
                self.finger_joint_indices.append(jid)
            elif joint_type == p.JOINT_REVOLUTE and len(self.arm_joint_indices) < 6:
                self.arm_joint_indices.append(jid)

        self.backend = p
        print(f"[OK] Simulation backend ready with {len(self.arm_joint_indices)} arm joints")

        if len(self.finger_joint_indices) > 0:
            self.gripper_controller = GripperController(
                self._robot_id, self.finger_joint_indices, dt=1 / 120.0
            )
            print("[OK] Gripper controller initialized")
        else:
            print("[WARN] No gripper joints found in URDF")

    def _init_real_robot(self, robot_ip: str):
        """Initialize real robot backend via SDK RPC."""
        try:
            import Robot
        except ImportError as e:
            raise ImportError(f"Failed to import Robot SDK module: {e}")

        try:
            self.backend = Robot.RPC(robot_ip)
            print(f"[OK] Connected to real FR5 robot at {robot_ip}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to robot at {robot_ip}: {e}")

    def move_j(self, joint_positions: List[float], velocity: float = 50, acceleration: float = 100) -> int:
        """Move robot to target joint positions (degrees)."""
        if len(joint_positions) != 6:
            print(f"Error: Expected 6 joint positions, got {len(joint_positions)}")
            return -1

        if self.mode == "simulation":
            for idx, jid in enumerate(self.arm_joint_indices[:6]):
                self._p.resetJointState(self._robot_id, jid, math.radians(float(joint_positions[idx])))
            self._p.stepSimulation()
            return 0

        return self.backend.MoveJ(joint_pos=joint_positions, tool=0, user=0, vel=velocity)

    def move_l(self, cartesian_pose: List[float], velocity: float = 50, acceleration: float = 100) -> int:
        """Move robot linearly to Cartesian pose [x, y, z, rx, ry, rz]."""
        if len(cartesian_pose) != 6:
            print(f"Error: Expected 6 cartesian values [x,y,z,rx,ry,rz], got {len(cartesian_pose)}")
            return -1

        if self.mode == "simulation":
            print("[WARN] MoveL not implemented for simulation backend")
            return -1

        return self.backend.MoveL(desc_pos=cartesian_pose, tool=0, user=0, vel=velocity)

    def move_gripper(
        self, position_pct: float, speed_pct: float = 50, force_pct: float = 50, block: bool = True
    ) -> int:
        """Move gripper to percentage opening (0=closed, 100=open)."""
        position_pct = float(np.clip(position_pct, 0, 100))

        if self.mode == "simulation":
            if self.gripper_controller is None:
                print("Error: No gripper available in simulation")
                return -1

            duration = 1.0 * (100 / max(float(speed_pct), 10.0))
            success = self.gripper_controller.set_position_percent(position_pct, duration=duration)
            return 0 if success else -1

        return self.backend.MoveGripper(
            1,
            position_pct,
            speed_pct,
            force_pct,
            5000,
            1 if block else 0,
            0,
            0,
            0,
            0,
        )

    def open_gripper(self, speed_pct: float = 50) -> int:
        """Open gripper fully."""
        return self.move_gripper(100, speed_pct=speed_pct)

    def close_gripper(self, speed_pct: float = 50, force_pct: float = 50) -> int:
        """Close gripper fully."""
        return self.move_gripper(0, speed_pct=speed_pct, force_pct=force_pct)

    def get_state(self) -> Dict:
        """Get current robot state."""
        state = {"mode": self.mode}

        if self.mode == "simulation":
            joint_deg = []
            for jid in self.arm_joint_indices[:6]:
                joint_deg.append(math.degrees(self._p.getJointState(self._robot_id, jid)[0]))
            state["joint_positions"] = joint_deg
            state["tcp_pose"] = [0, 0, 0, 0, 0, 0]
            state["gripper_state"] = (
                self.gripper_controller.get_state() if self.gripper_controller is not None else None
            )
            return state

        state["joint_positions"] = list(self.backend.robot_state_pkg.jt_cur_pos)
        state["tcp_pose"] = list(self.backend.robot_state_pkg.tl_cur_pos)
        state["gripper_state"] = {
            "position": self.backend.robot_state_pkg.gripper_cur_pos,
            "speed": self.backend.robot_state_pkg.gripper_cur_speed,
            "current": self.backend.robot_state_pkg.gripper_cur_current,
        }
        return state

    def get_joint_positions(self) -> List[float]:
        """Get current joint positions in degrees."""
        return self.get_state()["joint_positions"]

    def check_workspace_limits(self, joint_positions: List[float]) -> Tuple[bool, str]:
        """Check whether a joint target is inside joint-space limits."""
        from workspace_boundaries import WorkspaceBoundary

        return WorkspaceBoundary.check_joint_limits(joint_positions)

    def disconnect(self):
        """Close active backend and release resources."""
        if self.mode == "simulation":
            if self._p is not None and self._physics_client is not None:
                if self._p.isConnected(self._physics_client):
                    self._p.disconnect(self._physics_client)
            if self._patched_urdf and os.path.exists(self._patched_urdf):
                try:
                    os.unlink(self._patched_urdf)
                except OSError:
                    pass
        else:
            self.backend.CloseRPC()

        print("[OK] FR5 robot disconnected")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
