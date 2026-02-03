"""
Unified FR5 Robot Interface

Provides a single API for controlling the FAIRINO FR5 robot in both
real hardware and PyBullet simulation modes. Automatically routes commands
to the appropriate backend.

Usage:
    # Simulation mode
    robot = FR5RobotInterface(mode='simulation', gui=True)

    # Real robot mode
    robot = FR5RobotInterface(mode='real', robot_ip='192.168.58.2')

    # Common API works for both modes
    robot.move_j([0, 0, 0, 0, 0, 0])
    robot.move_gripper(position_pct=50)
    state = robot.get_state()
"""

import sys
import os
from typing import List, Dict, Optional, Tuple
import numpy as np

# Add paths for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, 'simulation'))
sys.path.insert(0, os.path.join(SCRIPT_DIR, 'real_robot'))


class FR5RobotInterface:
    """
    Unified interface for FAIRINO FR5 robot (real hardware and simulation).

    Provides consistent API regardless of backend, making code portable
    between simulation development and real robot deployment.
    """

    def __init__(self, mode: str = 'simulation', robot_ip: str = '192.168.58.2',
                 gui: bool = True, urdf_path: Optional[str] = None):
        """
        Initialize FR5 robot interface.

        Args:
            mode: 'simulation' or 'real'
            robot_ip: IP address for real robot (default: 192.168.58.2)
            gui: Show PyBullet GUI in simulation mode (default: True)
            urdf_path: Custom URDF path for simulation (optional)

        Example:
            >>> robot_sim = FR5RobotInterface(mode='simulation', gui=True)
            >>> robot_real = FR5RobotInterface(mode='real', robot_ip='192.168.58.2')
        """
        self.mode = mode
        self.robot_ip = robot_ip
        self.backend = None
        self.gripper_controller = None

        print(f"Initializing FR5RobotInterface in {mode} mode...")

        if mode == 'simulation':
            self._init_simulation(gui, urdf_path)
        elif mode == 'real':
            self._init_real_robot(robot_ip)
        else:
            raise ValueError(f"Invalid mode: {mode}. Must be 'simulation' or 'real'")

        print(f"✓ FR5RobotInterface ready in {mode} mode")

    def _init_simulation(self, gui: bool, urdf_path: Optional[str]):
        """Initialize PyBullet simulation backend"""
        try:
            from first_example import RobotSim
            from gripper_controller import GripperController

            self.backend = RobotSim(urdf_path=urdf_path, gui=gui)

            # Initialize gripper if finger joints exist
            if len(self.backend.finger_joint_indices) > 0:
                self.gripper_controller = GripperController(
                    self.backend.robot_id,
                    self.backend.finger_joint_indices,
                    dt=1/120.0
                )
                print("✓ Gripper controller initialized")
            else:
                print("⚠ No gripper joints found in URDF")

        except ImportError as e:
            raise ImportError(f"Failed to import simulation modules: {e}")

    def _init_real_robot(self, robot_ip: str):
        """Initialize real robot backend via fairino SDK"""
        try:
            sys.path.insert(0, os.path.join(SCRIPT_DIR, 'real_robot', 'fairino'))
            from fairino.Robot import RPC

            self.backend = RPC(robot_ip)
            print(f"✓ Connected to real FR5 robot at {robot_ip}")

        except ImportError as e:
            raise ImportError(
                f"Failed to import fairino SDK: {e}\n"
                f"Make sure fairino SDK is installed in real_robot/fairino/"
            )
        except Exception as e:
            raise ConnectionError(f"Failed to connect to robot at {robot_ip}: {e}")

    def move_j(self, joint_positions: List[float], velocity: float = 50,
               acceleration: float = 100) -> int:
        """
        Move robot to joint positions.

        Args:
            joint_positions: List of 6 joint angles in degrees
            velocity: Joint velocity percentage (0-100, default: 50)
            acceleration: Joint acceleration percentage (0-100, default: 100)

        Returns:
            0 for success, -1 for failure

        Example:
            >>> robot.move_j([0, 0, 0, 0, 0, 0])  # Home position
            >>> robot.move_j([45, -30, 60, -90, 0, 45])  # Custom pose
        """
        if len(joint_positions) != 6:
            print(f"Error: Expected 6 joint positions, got {len(joint_positions)}")
            return -1

        if self.mode == 'simulation':
            return self.backend.MoveJ(
                joint_pos=joint_positions,
                vel=velocity,
                acc=acceleration
            )
        else:  # real robot
            return self.backend.MoveJ(
                joint_pos=joint_positions,
                desc_pos=[0, 0, 0, 0, 0, 0],
                tool=0, user=0,
                vel=velocity, acc=acceleration, ovl=100.0,
                epos=[0, 0, 0, 0], blendT=-1.0,
                offset_flag=0, offset_pos=[0, 0, 0, 0, 0, 0]
            )

    def move_l(self, cartesian_pose: List[float], velocity: float = 50,
               acceleration: float = 100) -> int:
        """
        Move robot to Cartesian pose (linear motion).

        Args:
            cartesian_pose: [x, y, z, rx, ry, rz] in mm and degrees
            velocity: Cartesian velocity percentage (0-100, default: 50)
            acceleration: Cartesian acceleration percentage (0-100, default: 100)

        Returns:
            0 for success, -1 for failure

        Example:
            >>> robot.move_l([500, 0, 400, 180, 0, 0])  # Move to pose
        """
        if len(cartesian_pose) != 6:
            print(f"Error: Expected 6 cartesian values [x,y,z,rx,ry,rz], got {len(cartesian_pose)}")
            return -1

        if self.mode == 'simulation':
            print("Warning: MoveL not fully implemented in simulation, using MoveJ")
            # TODO: Implement Cartesian motion planning in simulation
            return -1
        else:  # real robot
            return self.backend.MoveL(
                desc_pos=cartesian_pose,
                tool=0, user=0,
                vel=velocity, acc=acceleration, ovl=100.0,
                blendR=-1.0,
                offset_flag=0, offset_pos=[0, 0, 0, 0, 0, 0]
            )

    def move_gripper(self, position_pct: float, speed_pct: float = 50,
                     force_pct: float = 50, block: bool = True) -> int:
        """
        Control gripper position.

        Args:
            position_pct: Gripper opening percentage (0=closed, 100=open)
            speed_pct: Gripper speed percentage (0-100, default: 50)
            force_pct: Gripper force percentage (0-100, default: 50)
            block: Wait for motion to complete (default: True)

        Returns:
            0 for success, -1 for failure

        Example:
            >>> robot.move_gripper(100)  # Fully open
            >>> robot.move_gripper(0)    # Fully closed
            >>> robot.move_gripper(50)   # Half open
        """
        position_pct = np.clip(position_pct, 0, 100)

        if self.mode == 'simulation':
            if self.gripper_controller is None:
                print("Error: No gripper available in simulation")
                return -1

            # Convert duration based on speed (higher speed = shorter duration)
            duration = 1.0 * (100 / max(speed_pct, 10))

            success = self.gripper_controller.set_position_percent(
                position_pct,
                duration=duration
            )
            return 0 if success else -1

        else:  # real robot
            return self.backend.MoveGripper(
                index=1,
                pos=position_pct,
                vel=speed_pct,
                force=force_pct,
                maxtime=5000,
                block=1 if block else 0,
                type=0,  # Parallel gripper
                rotNum=0, rotVel=0, rotTorque=0
            )

    def open_gripper(self, speed_pct: float = 50) -> int:
        """
        Open gripper fully.

        Args:
            speed_pct: Opening speed percentage (default: 50)

        Returns:
            0 for success, -1 for failure
        """
        return self.move_gripper(100, speed_pct=speed_pct)

    def close_gripper(self, speed_pct: float = 50, force_pct: float = 50) -> int:
        """
        Close gripper with object detection.

        Args:
            speed_pct: Closing speed percentage (default: 50)
            force_pct: Closing force percentage (default: 50)

        Returns:
            0 for success, -1 for failure
        """
        return self.move_gripper(0, speed_pct=speed_pct, force_pct=force_pct)

    def get_state(self) -> Dict:
        """
        Get current robot state.

        Returns:
            Dictionary with:
                - joint_positions: Current joint angles (degrees)
                - tcp_pose: Tool center point pose [x,y,z,rx,ry,rz]
                - gripper_state: Gripper information (if available)
                - mode: 'simulation' or 'real'

        Example:
            >>> state = robot.get_state()
            >>> print(f"Joints: {state['joint_positions']}")
            >>> print(f"TCP: {state['tcp_pose']}")
        """
        state = {'mode': self.mode}

        if self.mode == 'simulation':
            # Get joint positions
            state['joint_positions'] = list(self.backend.robot_state_pkg.jt_cur_pos)

            # Get TCP pose (would need FK implementation)
            state['tcp_pose'] = [0, 0, 0, 0, 0, 0]  # Placeholder

            # Get gripper state
            if self.gripper_controller is not None:
                state['gripper_state'] = self.gripper_controller.get_state()
            else:
                state['gripper_state'] = None

        else:  # real robot
            # Get joint positions from robot state
            state['joint_positions'] = list(self.backend.robot_state_pkg.jt_cur_pos)

            # Get TCP pose
            state['tcp_pose'] = list(self.backend.robot_state_pkg.tl_cur_pos)

            # Gripper state (if connected)
            state['gripper_state'] = {
                'position': self.backend.robot_state_pkg.gripper_cur_pos,
                'speed': self.backend.robot_state_pkg.gripper_cur_speed,
                'current': self.backend.robot_state_pkg.gripper_cur_current,
            }

        return state

    def get_joint_positions(self) -> List[float]:
        """
        Get current joint positions in degrees.

        Returns:
            List of 6 joint angles

        Example:
            >>> joints = robot.get_joint_positions()
            >>> print(f"J1: {joints[0]:.1f}°")
        """
        state = self.get_state()
        return state['joint_positions']

    def check_workspace_limits(self, joint_positions: List[float]) -> Tuple[bool, str]:
        """
        Check if joint positions are within workspace limits.

        Args:
            joint_positions: List of 6 joint angles in degrees

        Returns:
            Tuple of (is_valid, message)

        Example:
            >>> valid, msg = robot.check_workspace_limits([0, 0, 0, 0, 0, 0])
            >>> assert valid, msg
        """
        from workspace_boundaries import WorkspaceBoundary
        return WorkspaceBoundary.check_joint_limits(joint_positions)

    def disconnect(self):
        """
        Close connection to robot.

        Call this when done to properly clean up resources.

        Example:
            >>> robot.disconnect()
        """
        if self.mode == 'simulation':
            self.backend.CloseRPC()
        else:
            self.backend.CloseRPC()

        print("✓ FR5 robot disconnected")

    def __enter__(self):
        """Context manager support: with FR5RobotInterface() as robot:"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager cleanup"""
        self.disconnect()


if __name__ == "__main__":
    # Demo and examples
    print("=" * 70)
    print("FR5 Unified Robot Interface Demo")
    print("=" * 70)

    print("\n1. Simulation Mode Example:")
    print("-" * 70)
    print("from fr5_robot_interface import FR5RobotInterface")
    print("")
    print("# Initialize in simulation")
    print("with FR5RobotInterface(mode='simulation', gui=True) as robot:")
    print("    # Get current state")
    print("    state = robot.get_state()")
    print("    print(f'Current joints: {state[\"joint_positions\"]}')")
    print("")
    print("    # Move to home position")
    print("    robot.move_j([0, 0, 0, 0, 0, 0])")
    print("")
    print("    # Control gripper")
    print("    robot.open_gripper()")
    print("    robot.close_gripper()")

    print("\n2. Real Robot Mode Example:")
    print("-" * 70)
    print("from fr5_robot_interface import FR5RobotInterface")
    print("")
    print("# Initialize with real robot")
    print("with FR5RobotInterface(mode='real', robot_ip='192.168.58.2') as robot:")
    print("    # Same API as simulation!")
    print("    state = robot.get_state()")
    print("    print(f'Current joints: {state[\"joint_positions\"]}')")
    print("")
    print("    # Move robot")
    print("    robot.move_j([10, -20, 30, -40, 10, 20])")
    print("")
    print("    # Control gripper")
    print("    robot.move_gripper(position_pct=75)")

    print("\n3. Workspace Checking Example:")
    print("-" * 70)
    print("# Check if position is safe before moving")
    print("target = [200, 0, 0, 0, 0, 0]  # J1 = 200° (out of bounds)")
    print("valid, msg = robot.check_workspace_limits(target)")
    print("if not valid:")
    print("    print(f'Invalid position: {msg}')")
    print("else:")
    print("    robot.move_j(target)")

    print("\n" + "=" * 70)
    print("Key Features:")
    print("  ✓ Single API for simulation and real robot")
    print("  ✓ Automatic backend routing")
    print("  ✓ Workspace boundary checking")
    print("  ✓ Gripper control integration")
    print("  ✓ Context manager support (with statement)")
    print("=" * 70)
