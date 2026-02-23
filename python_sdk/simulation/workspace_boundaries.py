"""
Workspace Boundary Management for FR5 Robot

Defines and enforces workspace boundaries for the FAIRINO FR5 collaborative robot.
Includes joint limit checking, Cartesian workspace envelope validation, and
visual boundary display in PyBullet.

FR5 Robot Specifications:
- 6-axis collaborative robot arm
- Reach: 922mm (0.922m)
- Payload: 5kg
- Repeatability: ±0.02mm
"""

import numpy as np
from typing import Tuple, List, Optional
import pybullet as p


class WorkspaceBoundary:
    """
    FR5 workspace boundary definition and checking.

    This class provides methods for validating joint positions and Cartesian
    positions against the robot's physical limitations and safety constraints.
    """

    # FR5 Joint Limits (from URDF specifications) in degrees
    JOINT_LIMITS = [
        (-175, 175),   # J1: Base rotation, ±175°
        (-265, 85),    # J2: Shoulder, -265° to +85°
        (-162, 162),   # J3: Elbow, ±162°
        (-265, 85),    # J4: Wrist roll, -265° to +85°
        (-175, 175),   # J5: Wrist pitch, ±175°
        (-175, 175),   # J6: Wrist yaw, ±175°
    ]

    # Cartesian workspace bounds (meters, from robot base frame)
    CARTESIAN_LIMITS = {
        'x': (-0.9, 0.9),       # Forward/backward range
        'y': (-0.9, 0.9),       # Left/right range
        'z': (0.0, 1.2),        # Vertical range (above base)
        'radius_min': 0.15,     # Minimum reach (safety zone around base)
        'radius_max': 0.85,     # Maximum reach (conservative, nominal 0.922m)
    }

    @classmethod
    def check_joint_limits(cls, joint_angles_deg: List[float]) -> Tuple[bool, str]:
        """
        Check if joint angles are within safe limits.

        Args:
            joint_angles_deg: List of 6 joint angles in degrees

        Returns:
            Tuple of (is_valid, message):
                - is_valid: True if all joints within limits, False otherwise
                - message: "OK" if valid, error description if invalid

        Example:
            >>> valid, msg = WorkspaceBoundary.check_joint_limits([0, 0, 0, 0, 0, 0])
            >>> assert valid, msg
            >>> valid, msg = WorkspaceBoundary.check_joint_limits([200, 0, 0, 0, 0, 0])
            >>> assert not valid  # J1 exceeds ±175°
        """
        if len(joint_angles_deg) != 6:
            return False, f"Expected 6 joint angles, got {len(joint_angles_deg)}"

        for i, (angle, (lo, hi)) in enumerate(zip(joint_angles_deg, cls.JOINT_LIMITS)):
            if angle < lo or angle > hi:
                return False, f"Joint {i+1} out of range: {angle:.1f}° not in [{lo}, {hi}]°"

        return True, "OK"

    @classmethod
    def clip_joint_to_limits(cls, joint_angles_deg: List[float]) -> np.ndarray:
        """
        Clip joint angles to within safe limits.

        Args:
            joint_angles_deg: List of 6 joint angles in degrees

        Returns:
            np.ndarray: Clipped joint angles within limits

        Example:
            >>> angles = [200, 0, 0, 0, 0, 0]  # J1 exceeds limit
            >>> clipped = WorkspaceBoundary.clip_joint_to_limits(angles)
            >>> assert clipped[0] == 175  # Clipped to max limit
        """
        clipped = []
        for angle, (lo, hi) in zip(joint_angles_deg, cls.JOINT_LIMITS):
            clipped.append(np.clip(angle, lo, hi))
        return np.array(clipped)

    @classmethod
    def check_cartesian_position(cls, position: List[float]) -> Tuple[bool, str]:
        """
        Check if TCP (Tool Center Point) position is within reachable workspace.

        Args:
            position: [x, y, z] position in meters from robot base

        Returns:
            Tuple of (is_valid, message):
                - is_valid: True if position is reachable, False otherwise
                - message: "OK" if valid, error description if invalid

        Example:
            >>> valid, msg = WorkspaceBoundary.check_cartesian_position([0.5, 0.0, 0.5])
            >>> assert valid, msg
            >>> valid, msg = WorkspaceBoundary.check_cartesian_position([1.5, 0.0, 0.5])
            >>> assert not valid  # Beyond max reach
        """
        if len(position) != 3:
            return False, f"Expected [x, y, z] position, got {len(position)} values"

        x, y, z = position
        radius = np.sqrt(x**2 + y**2)

        # Check Cartesian bounds
        if not (cls.CARTESIAN_LIMITS['x'][0] <= x <= cls.CARTESIAN_LIMITS['x'][1]):
            return False, f"X position {x:.3f}m out of bounds {cls.CARTESIAN_LIMITS['x']}"

        if not (cls.CARTESIAN_LIMITS['y'][0] <= y <= cls.CARTESIAN_LIMITS['y'][1]):
            return False, f"Y position {y:.3f}m out of bounds {cls.CARTESIAN_LIMITS['y']}"

        if not (cls.CARTESIAN_LIMITS['z'][0] <= z <= cls.CARTESIAN_LIMITS['z'][1]):
            return False, f"Z position {z:.3f}m out of bounds {cls.CARTESIAN_LIMITS['z']}"

        # Check radial reach constraints
        if radius < cls.CARTESIAN_LIMITS['radius_min']:
            return False, f"Position too close to base: {radius:.3f}m < {cls.CARTESIAN_LIMITS['radius_min']}m"

        if radius > cls.CARTESIAN_LIMITS['radius_max']:
            return False, f"Position beyond reach: {radius:.3f}m > {cls.CARTESIAN_LIMITS['radius_max']}m"

        return True, "OK"

    @classmethod
    def visualize_workspace(cls, physics_client=None) -> List[int]:
        """
        Draw workspace boundary visualization in PyBullet.

        Creates visual debug lines showing:
        - Outer boundary cylinder (red): Maximum reach
        - Inner boundary cylinder (yellow): Minimum reach (safety zone)
        - Height limits (top and bottom planes)

        Args:
            physics_client: PyBullet physics client (optional, uses default if None)

        Returns:
            List of debug line IDs for later removal if needed

        Example:
            >>> import pybullet as p
            >>> p.connect(p.GUI)
            >>> line_ids = WorkspaceBoundary.visualize_workspace()
            >>> # ... simulation runs ...
            >>> for line_id in line_ids:
            ...     p.removeUserDebugItem(line_id)
        """
        line_ids = []
        num_segments = 36  # Number of segments for circular boundary

        # Draw cylindrical workspace boundaries
        for i in range(num_segments):
            angle = 2 * np.pi * i / num_segments
            angle_next = 2 * np.pi * (i + 1) / num_segments

            # Outer boundary (maximum reach)
            r_max = cls.CARTESIAN_LIMITS['radius_max']

            # Bottom circle
            p1_bottom = [r_max * np.cos(angle), r_max * np.sin(angle), cls.CARTESIAN_LIMITS['z'][0]]
            p2_bottom = [r_max * np.cos(angle_next), r_max * np.sin(angle_next), cls.CARTESIAN_LIMITS['z'][0]]
            line_ids.append(p.addUserDebugLine(p1_bottom, p2_bottom, [1, 0, 0], 2))  # Red

            # Top circle
            p1_top = [r_max * np.cos(angle), r_max * np.sin(angle), cls.CARTESIAN_LIMITS['z'][1]]
            p2_top = [r_max * np.cos(angle_next), r_max * np.sin(angle_next), cls.CARTESIAN_LIMITS['z'][1]]
            line_ids.append(p.addUserDebugLine(p1_top, p2_top, [1, 0, 0], 2))  # Red

            # Vertical lines (every 6 segments)
            if i % 6 == 0:
                line_ids.append(p.addUserDebugLine(p1_bottom, p1_top, [1, 0, 0], 1))  # Red

            # Inner boundary (minimum reach - safety zone)
            r_min = cls.CARTESIAN_LIMITS['radius_min']

            # Bottom circle
            p1_bottom_inner = [r_min * np.cos(angle), r_min * np.sin(angle), cls.CARTESIAN_LIMITS['z'][0]]
            p2_bottom_inner = [r_min * np.cos(angle_next), r_min * np.sin(angle_next), cls.CARTESIAN_LIMITS['z'][0]]
            line_ids.append(p.addUserDebugLine(p1_bottom_inner, p2_bottom_inner, [1, 1, 0], 2))  # Yellow

            # Top circle
            p1_top_inner = [r_min * np.cos(angle), r_min * np.sin(angle), cls.CARTESIAN_LIMITS['z'][1]]
            p2_top_inner = [r_min * np.cos(angle_next), r_min * np.sin(angle_next), cls.CARTESIAN_LIMITS['z'][1]]
            line_ids.append(p.addUserDebugLine(p1_top_inner, p2_top_inner, [1, 1, 0], 2))  # Yellow

        # Draw Cartesian box boundaries (X and Y limits)
        x_min, x_max = cls.CARTESIAN_LIMITS['x']
        y_min, y_max = cls.CARTESIAN_LIMITS['y']
        z_min, z_max = cls.CARTESIAN_LIMITS['z']

        # Draw box corners (green)
        corners = [
            [x_min, y_min, z_min], [x_max, y_min, z_min],
            [x_max, y_max, z_min], [x_min, y_max, z_min],
            [x_min, y_min, z_max], [x_max, y_min, z_max],
            [x_max, y_max, z_max], [x_min, y_max, z_max],
        ]

        # Bottom square
        for i in range(4):
            line_ids.append(p.addUserDebugLine(corners[i], corners[(i+1) % 4], [0, 1, 0], 1))

        # Top square
        for i in range(4):
            line_ids.append(p.addUserDebugLine(corners[i+4], corners[(i+1) % 4 + 4], [0, 1, 0], 1))

        # Vertical edges
        for i in range(4):
            line_ids.append(p.addUserDebugLine(corners[i], corners[i+4], [0, 1, 0], 1))

        return line_ids

    @classmethod
    def get_workspace_info(cls) -> str:
        """
        Get human-readable workspace information.

        Returns:
            str: Formatted workspace boundary information

        Example:
            >>> print(WorkspaceBoundary.get_workspace_info())
            FR5 Workspace Boundaries
            ========================
            ...
        """
        info = []
        info.append("FR5 Workspace Boundaries")
        info.append("=" * 50)
        info.append("\nJoint Limits (degrees):")
        for i, (lo, hi) in enumerate(cls.JOINT_LIMITS):
            info.append(f"  J{i+1}: [{lo:4.0f}°, {hi:4.0f}°]")

        info.append("\nCartesian Workspace (meters):")
        info.append(f"  X range: [{cls.CARTESIAN_LIMITS['x'][0]:.2f}, {cls.CARTESIAN_LIMITS['x'][1]:.2f}]")
        info.append(f"  Y range: [{cls.CARTESIAN_LIMITS['y'][0]:.2f}, {cls.CARTESIAN_LIMITS['y'][1]:.2f}]")
        info.append(f"  Z range: [{cls.CARTESIAN_LIMITS['z'][0]:.2f}, {cls.CARTESIAN_LIMITS['z'][1]:.2f}]")
        info.append(f"  Radial reach: [{cls.CARTESIAN_LIMITS['radius_min']:.2f}m, {cls.CARTESIAN_LIMITS['radius_max']:.2f}m]")
        info.append("=" * 50)

        return "\n".join(info)


if __name__ == "__main__":
    # Demo and validation
    print(WorkspaceBoundary.get_workspace_info())
    print("\nRunning validation tests...")

    # Test 1: Valid joint positions
    print("\nTest 1: Valid joint positions")
    valid_joints = [0, 0, 0, 0, 0, 0]
    result, msg = WorkspaceBoundary.check_joint_limits(valid_joints)
    assert result, f"Test failed: {msg}"
    print(f"✓ {valid_joints} -> {msg}")

    # Test 2: Invalid joint positions
    print("\nTest 2: Invalid joint positions")
    invalid_joints = [200, 0, 0, 0, 0, 0]  # J1 exceeds ±175°
    result, msg = WorkspaceBoundary.check_joint_limits(invalid_joints)
    assert not result, "Should have failed"
    print(f"✓ {invalid_joints} -> {msg}")

    # Test 3: Joint clipping
    print("\nTest 3: Joint clipping")
    clipped = WorkspaceBoundary.clip_joint_to_limits(invalid_joints)
    print(f"✓ {invalid_joints} -> {clipped.tolist()}")
    assert clipped[0] == 175, "Should be clipped to max limit"

    # Test 4: Valid Cartesian position
    print("\nTest 4: Valid Cartesian position")
    valid_pos = [0.5, 0.0, 0.5]
    result, msg = WorkspaceBoundary.check_cartesian_position(valid_pos)
    assert result, f"Test failed: {msg}"
    print(f"✓ {valid_pos} -> {msg}")

    # Test 5: Position beyond reach
    print("\nTest 5: Position beyond reach")
    far_pos = [1.0, 0.0, 0.5]  # Beyond 0.85m radius
    result, msg = WorkspaceBoundary.check_cartesian_position(far_pos)
    assert not result, "Should have failed"
    print(f"✓ {far_pos} -> {msg}")

    # Test 6: Position too close to base
    print("\nTest 6: Position too close to base")
    close_pos = [0.05, 0.05, 0.3]  # < 0.15m radius
    result, msg = WorkspaceBoundary.check_cartesian_position(close_pos)
    assert not result, "Should have failed"
    print(f"✓ {close_pos} -> {msg}")

    print("\n" + "=" * 50)
    print("All workspace boundary tests passed! ✓")
