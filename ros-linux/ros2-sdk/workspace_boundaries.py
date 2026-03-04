"""
Workspace Boundary Management for FR5 Robot (Gripper-Aware) — ROS2 Version

Defines and enforces workspace boundaries for the FAIRINO FR5 collaborative robot.

FR5 Robot Specifications:
- Reach: 922mm (0.922m), Payload: 5kg, Repeatability: +/-0.02mm

AG-95 Gripper Extension:
- Adds 280mm total (0.1m coupler + 0.18m to grasp frame)
- Effective reach with gripper: ~1.2m (conservative max 1.1m)

Note: PyBullet visualization is not available in ROS2. Use RViz markers instead.
"""

import numpy as np
from typing import Tuple, List


class WorkspaceBoundary:
    """
    FR5 workspace boundary definition and checking.

    Supports both bare FR5 and FR5 with AG-95 gripper (default).
    """

    JOINT_LIMITS = [
        (-175, 175),
        (-265, 85),
        (-162, 162),
        (-265, 85),
        (-175, 175),
        (-175, 175),
    ]

    GRIPPER_OFFSET = 0.28

    CARTESIAN_LIMITS = {
        'x': (-0.9, 0.9),
        'y': (-0.9, 0.9),
        'z': (0.0, 1.2),
        'radius_min': 0.15,
        'radius_max': 0.85,
    }

    CARTESIAN_LIMITS_WITH_GRIPPER = {
        'x': (-1.0, 1.0),
        'y': (-1.0, 1.0),
        'z': (0.0, 1.4),
        'radius_min': 0.20,
        'radius_max': 1.05,
    }

    @classmethod
    def _limits(cls, with_gripper=True):
        return cls.CARTESIAN_LIMITS_WITH_GRIPPER if with_gripper else cls.CARTESIAN_LIMITS

    @classmethod
    def check_joint_limits(cls, joint_angles_deg: List[float]) -> Tuple[bool, str]:
        """Check if joint angles are within safe limits."""
        if len(joint_angles_deg) != 6:
            return False, f"Expected 6 joint angles, got {len(joint_angles_deg)}"

        for i, (angle, (lo, hi)) in enumerate(zip(joint_angles_deg, cls.JOINT_LIMITS)):
            if angle < lo or angle > hi:
                return False, f"Joint {i+1} out of range: {angle:.1f} deg not in [{lo}, {hi}]"

        return True, "OK"

    @classmethod
    def clip_joint_to_limits(cls, joint_angles_deg: List[float]) -> np.ndarray:
        """Clip joint angles to within safe limits."""
        return np.array([np.clip(a, lo, hi) for a, (lo, hi) in zip(joint_angles_deg, cls.JOINT_LIMITS)])

    @classmethod
    def check_cartesian_position(cls, position: List[float], with_gripper: bool = True) -> Tuple[bool, str]:
        """
        Check if TCP position is within reachable workspace.

        Args:
            position: [x, y, z] in meters from robot base
            with_gripper: Use gripper-aware limits (default True)
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
    def visualize_workspace(cls, with_gripper: bool = True, physics_client=None):
        """Not available in ROS2. Use RViz markers for visualization."""
        raise NotImplementedError(
            "PyBullet visualization not available in ROS2. "
            "Use RViz markers (visualization_msgs/Marker) instead."
        )

    @classmethod
    def get_workspace_info(cls, with_gripper: bool = True) -> str:
        """Get human-readable workspace boundary information."""
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
