"""
Workspace Boundary Management for FR5 Robot (Gripper-Aware)

Defines and enforces workspace boundaries for the FAIRINO FR5 collaborative robot.
Includes joint limit checking, Cartesian workspace envelope validation, and
visual boundary display in PyBullet.

FR5 Robot Specifications:
- 6-axis collaborative robot arm
- Reach: 922mm (0.922m)
- Payload: 5kg
- Repeatability: +/-0.02mm

AG-95 Gripper Extension:
- Adds 280mm total (0.1m coupler + 0.18m to grasp frame)
- Effective reach with gripper: ~1.2m (conservative max 1.1m)
"""

import numpy as np
from typing import Tuple, List, Optional
import pybullet as p


class WorkspaceBoundary:
    """
    FR5 workspace boundary definition and checking.

    Supports both bare FR5 and FR5 with AG-95 gripper (default).
    """

    # FR5 Joint Limits (from URDF specifications) in degrees
    JOINT_LIMITS = [
        (-175, 175),   # J1: Base rotation, +/-175 deg
        (-265, 85),    # J2: Shoulder, -265 to +85 deg
        (-162, 162),   # J3: Elbow, +/-162 deg
        (-265, 85),    # J4: Wrist roll, -265 to +85 deg
        (-175, 175),   # J5: Wrist pitch, +/-175 deg
        (-175, 175),   # J6: Wrist yaw, +/-175 deg
    ]

    # AG-95 gripper adds 280mm (0.1m coupler + 0.18m to grasp frame)
    GRIPPER_OFFSET = 0.28

    # Cartesian workspace bounds WITHOUT gripper (meters, robot base frame)
    CARTESIAN_LIMITS = {
        'x': (-0.9, 0.9),
        'y': (-0.9, 0.9),
        'z': (0.0, 1.2),
        'radius_min': 0.15,
        'radius_max': 0.85,
    }

    # Cartesian workspace bounds WITH AG-95 gripper
    CARTESIAN_LIMITS_WITH_GRIPPER = {
        'x': (-1.0, 1.0),
        'y': (-1.0, 1.0),
        'z': (0.0, 1.4),
        'radius_min': 0.20,   # Larger dead zone — gripper footprint
        'radius_max': 1.05,   # 0.85 + 0.28 = 1.13, conservative 1.05
    }

    @classmethod
    def _limits(cls, with_gripper=True):
        """Return the appropriate CARTESIAN_LIMITS dict."""
        return cls.CARTESIAN_LIMITS_WITH_GRIPPER if with_gripper else cls.CARTESIAN_LIMITS

    @classmethod
    def check_joint_limits(cls, joint_angles_deg: List[float]) -> Tuple[bool, str]:
        """
        Check if joint angles are within safe limits.

        Args:
            joint_angles_deg: List of 6 joint angles in degrees

        Returns:
            Tuple of (is_valid, message)
        """
        if len(joint_angles_deg) != 6:
            return False, f"Expected 6 joint angles, got {len(joint_angles_deg)}"

        for i, (angle, (lo, hi)) in enumerate(zip(joint_angles_deg, cls.JOINT_LIMITS)):
            if angle < lo or angle > hi:
                return False, f"Joint {i+1} out of range: {angle:.1f} deg not in [{lo}, {hi}]"

        return True, "OK"

    @classmethod
    def clip_joint_to_limits(cls, joint_angles_deg: List[float]) -> np.ndarray:
        """
        Clip joint angles to within safe limits.

        Args:
            joint_angles_deg: List of 6 joint angles in degrees

        Returns:
            np.ndarray: Clipped joint angles within limits
        """
        clipped = []
        for angle, (lo, hi) in zip(joint_angles_deg, cls.JOINT_LIMITS):
            clipped.append(np.clip(angle, lo, hi))
        return np.array(clipped)

    @classmethod
    def check_cartesian_position(cls, position: List[float], with_gripper: bool = True) -> Tuple[bool, str]:
        """
        Check if TCP position is within reachable workspace.

        Args:
            position: [x, y, z] position in meters from robot base
            with_gripper: Use gripper-aware limits if True (default True)

        Returns:
            Tuple of (is_valid, message)
        """
        if len(position) != 3:
            return False, f"Expected [x, y, z], got {len(position)} values"

        lim = cls._limits(with_gripper)
        x, y, z = position
        radius = np.sqrt(x**2 + y**2)

        if not (lim['x'][0] <= x <= lim['x'][1]):
            return False, f"X={x:.3f}m out of bounds {lim['x']}"

        if not (lim['y'][0] <= y <= lim['y'][1]):
            return False, f"Y={y:.3f}m out of bounds {lim['y']}"

        if not (lim['z'][0] <= z <= lim['z'][1]):
            return False, f"Z={z:.3f}m out of bounds {lim['z']}"

        if radius < lim['radius_min']:
            return False, f"Too close to base: {radius:.3f}m < {lim['radius_min']}m"

        if radius > lim['radius_max']:
            return False, f"Beyond reach: {radius:.3f}m > {lim['radius_max']}m"

        return True, "OK"

    @classmethod
    def visualize_workspace(cls, with_gripper: bool = True, physics_client=None) -> List[int]:
        """
        Draw workspace boundary visualization in PyBullet.

        Args:
            with_gripper: Use gripper-aware limits if True (default True)
            physics_client: PyBullet physics client (optional, uses default)

        Returns:
            List of debug line IDs
        """
        lim = cls._limits(with_gripper)
        line_ids = []
        num_segments = 36

        for i in range(num_segments):
            angle = 2 * np.pi * i / num_segments
            angle_next = 2 * np.pi * (i + 1) / num_segments

            r_max = lim['radius_max']
            z_min, z_max = lim['z']

            p1_bot = [r_max * np.cos(angle), r_max * np.sin(angle), z_min]
            p2_bot = [r_max * np.cos(angle_next), r_max * np.sin(angle_next), z_min]
            line_ids.append(p.addUserDebugLine(p1_bot, p2_bot, [1, 0, 0], 2))

            p1_top = [r_max * np.cos(angle), r_max * np.sin(angle), z_max]
            p2_top = [r_max * np.cos(angle_next), r_max * np.sin(angle_next), z_max]
            line_ids.append(p.addUserDebugLine(p1_top, p2_top, [1, 0, 0], 2))

            if i % 6 == 0:
                line_ids.append(p.addUserDebugLine(p1_bot, p1_top, [1, 0, 0], 1))

            r_min = lim['radius_min']
            p1_bi = [r_min * np.cos(angle), r_min * np.sin(angle), z_min]
            p2_bi = [r_min * np.cos(angle_next), r_min * np.sin(angle_next), z_min]
            line_ids.append(p.addUserDebugLine(p1_bi, p2_bi, [1, 1, 0], 2))

            p1_ti = [r_min * np.cos(angle), r_min * np.sin(angle), z_max]
            p2_ti = [r_min * np.cos(angle_next), r_min * np.sin(angle_next), z_max]
            line_ids.append(p.addUserDebugLine(p1_ti, p2_ti, [1, 1, 0], 2))

        x_min, x_max = lim['x']
        y_min, y_max = lim['y']
        z_min, z_max = lim['z']

        corners = [
            [x_min, y_min, z_min], [x_max, y_min, z_min],
            [x_max, y_max, z_min], [x_min, y_max, z_min],
            [x_min, y_min, z_max], [x_max, y_min, z_max],
            [x_max, y_max, z_max], [x_min, y_max, z_max],
        ]

        for i in range(4):
            line_ids.append(p.addUserDebugLine(corners[i], corners[(i+1) % 4], [0, 1, 0], 1))
        for i in range(4):
            line_ids.append(p.addUserDebugLine(corners[i+4], corners[(i+1) % 4 + 4], [0, 1, 0], 1))
        for i in range(4):
            line_ids.append(p.addUserDebugLine(corners[i], corners[i+4], [0, 1, 0], 1))

        return line_ids

    @classmethod
    def get_workspace_info(cls, with_gripper: bool = True) -> str:
        """
        Get human-readable workspace information.

        Args:
            with_gripper: Show gripper-aware limits if True (default True)

        Returns:
            Formatted workspace boundary string
        """
        lim = cls._limits(with_gripper)
        gripper_str = f" + AG-95 gripper ({cls.GRIPPER_OFFSET}m)" if with_gripper else ""
        info = [f"FR5{gripper_str} Workspace Boundaries", "=" * 50]

        info.append("\nJoint Limits (degrees):")
        for i, (lo, hi) in enumerate(cls.JOINT_LIMITS):
            info.append(f"  J{i+1}: [{lo:4.0f}, {hi:4.0f}] deg")

        info.append("\nCartesian Workspace (meters):")
        info.append(f"  X range: {lim['x']}")
        info.append(f"  Y range: {lim['y']}")
        info.append(f"  Z range: {lim['z']}")
        info.append(f"  Radial reach: [{lim['radius_min']:.2f}, {lim['radius_max']:.2f}] m")
        info.append("=" * 50)

        return "\n".join(info)


if __name__ == "__main__":
    print(WorkspaceBoundary.get_workspace_info(with_gripper=True))
    print()
    print(WorkspaceBoundary.get_workspace_info(with_gripper=False))

    print("\nRunning validation tests...")

    valid, msg = WorkspaceBoundary.check_joint_limits([0, 0, 0, 0, 0, 0])
    assert valid, msg
    print(f"Joint limits OK: {msg}")

    valid, msg = WorkspaceBoundary.check_cartesian_position([0.8, 0.0, 0.5], with_gripper=True)
    assert valid, msg
    print(f"Cartesian with gripper OK: {msg}")

    valid, msg = WorkspaceBoundary.check_cartesian_position([0.8, 0.0, 0.5], with_gripper=False)
    assert not valid, "Should fail without gripper limits"
    print(f"Cartesian without gripper correctly rejected: {msg}")

    print("All tests passed.")
