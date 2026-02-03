"""
Gripper Controller for FAIRINO FR5 Robot with DH AG-95 Parallel Gripper

Provides smooth gripper control with force feedback and object detection
for the DH Robotics AG-95 parallel gripper (95mm stroke, 45N per finger).

DH AG-95 Specifications:
- Stroke: 95mm (0-0.095m total opening)
- Force: Up to 45N per finger (90N total)
- Position Resolution: ~0.1mm
- Repeatability: ±0.02mm
- Communication: I/O or Modbus
"""

import pybullet as p
import time
import numpy as np
from typing import Optional, Dict, List
from trajectory_interpolator import TrajectoryInterpolator


class GripperController:
    """
    Controller for DH AG-95 parallel gripper with smooth motion and force feedback.

    The gripper consists of two parallel fingers that move symmetrically.
    Each finger moves 0-47.5mm from center (95mm total stroke).
    """

    # DH AG-95 Gripper specifications
    GRIPPER_STROKE_MM = 95  # Total opening in mm
    GRIPPER_STROKE_M = 0.095  # Total opening in meters
    MAX_FORCE_PER_FINGER = 45  # Newtons
    POSITION_RESOLUTION_MM = 0.1  # Position resolution

    # PyBullet simulation parameters
    DEFAULT_OPEN_FORCE = 40  # N - Force when opening
    DEFAULT_CLOSE_FORCE = 60  # N - Force when closing (higher for gripping)
    DEFAULT_HOLD_FORCE = 80  # N - Force when holding object

    def __init__(self, robot_id: int, finger_joint_indices: List[int], dt: float = 1/120.0):
        """
        Initialize gripper controller.

        Args:
            robot_id: PyBullet robot body ID
            finger_joint_indices: List of finger joint IDs (typically 2 for parallel gripper)
            dt: Simulation time step in seconds (default: 1/120s = 8.33ms)
        """
        self.robot_id = robot_id
        self.finger_joints = finger_joint_indices
        self.dt = dt

        # Gripper state
        self.gripper_fully_open = self.GRIPPER_STROKE_M / 2  # 0.0475m per finger
        self.gripper_closed = 0.0  # Fully closed
        self.current_position = self.gripper_fully_open

        # Force tracking
        self.contact_detected = False
        self.last_contact_force = 0.0

        print(f"GripperController initialized: DH AG-95")
        print(f"  Stroke: {self.GRIPPER_STROKE_MM}mm")
        print(f"  Finger joints: {self.finger_joints}")

    def open(self, duration: float = 1.0, force: Optional[float] = None) -> bool:
        """
        Open gripper smoothly to full width (95mm).

        Args:
            duration: Motion duration in seconds (default: 1.0s)
            force: Force to apply (N). If None, uses DEFAULT_OPEN_FORCE

        Returns:
            bool: True if motion completed successfully

        Example:
            >>> controller.open(duration=0.5)
            >>> state = controller.get_state()
            >>> assert state['is_open']
        """
        if force is None:
            force = self.DEFAULT_OPEN_FORCE

        return self._move_to_position(self.gripper_fully_open, duration, force)

    def close(self, duration: float = 1.0, force: Optional[float] = None,
              detect_contact: bool = True) -> bool:
        """
        Close gripper with object detection.

        If contact is detected before fully closing, stops motion to avoid
        damaging objects or gripper.

        Args:
            duration: Motion duration in seconds (default: 1.0s)
            force: Force to apply (N). If None, uses DEFAULT_CLOSE_FORCE
            detect_contact: Stop when object contact detected (default: True)

        Returns:
            bool: True if closed (or object grasped), False if failed

        Example:
            >>> success = controller.close(detect_contact=True)
            >>> state = controller.get_state()
            >>> if state['has_object']:
            ...     print("Object grasped!")
        """
        if force is None:
            force = self.DEFAULT_CLOSE_FORCE

        return self._move_to_position(
            self.gripper_closed,
            duration,
            force,
            detect_contact=detect_contact
        )

    def set_position_mm(self, position_mm: float, duration: float = 0.5,
                        force: Optional[float] = None) -> bool:
        """
        Set gripper to specific opening width in millimeters.

        Args:
            position_mm: Target opening width (0-95mm)
            duration: Motion duration in seconds (default: 0.5s)
            force: Force to apply (N). If None, uses DEFAULT_CLOSE_FORCE

        Returns:
            bool: True if motion completed successfully

        Example:
            >>> controller.set_position_mm(50)  # Half open (50mm)
            >>> controller.set_position_mm(20)  # Small opening (20mm)
        """
        # Clip to valid range
        position_mm = np.clip(position_mm, 0, self.GRIPPER_STROKE_MM)

        # Convert to meters per finger (half of total stroke)
        position_m_per_finger = (position_mm / 1000.0) / 2.0

        if force is None:
            force = self.DEFAULT_CLOSE_FORCE

        return self._move_to_position(position_m_per_finger, duration, force)

    def set_position_percent(self, percent: float, duration: float = 0.5,
                            force: Optional[float] = None) -> bool:
        """
        Set gripper opening as percentage (0% = closed, 100% = fully open).

        Args:
            percent: Opening percentage (0-100)
            duration: Motion duration in seconds (default: 0.5s)
            force: Force to apply (N). If None, uses DEFAULT_CLOSE_FORCE

        Returns:
            bool: True if motion completed successfully

        Example:
            >>> controller.set_position_percent(100)  # Fully open
            >>> controller.set_position_percent(50)   # Half open
            >>> controller.set_position_percent(0)    # Closed
        """
        # Clip to valid range
        percent = np.clip(percent, 0, 100)

        # Convert percentage to millimeters
        position_mm = (percent / 100.0) * self.GRIPPER_STROKE_MM

        return self.set_position_mm(position_mm, duration, force)

    def _move_to_position(self, target_pos_per_finger: float, duration: float,
                         force: float, detect_contact: bool = False) -> bool:
        """
        Internal method: Smooth gripper motion with minimum-jerk trajectory.

        Args:
            target_pos_per_finger: Target position per finger in meters
            duration: Motion duration in seconds
            force: Force to apply in Newtons
            detect_contact: Stop when contact detected

        Returns:
            bool: True if motion completed, False if failed or contact detected
        """
        start_pos = self.current_position
        num_steps = int(duration / self.dt)

        # Generate minimum-jerk trajectory for smooth motion
        trajectory = TrajectoryInterpolator.minimum_jerk(
            np.array([start_pos]),
            np.array([target_pos_per_finger]),
            num_steps
        )

        # Execute trajectory
        for i, pos_array in enumerate(trajectory):
            pos = float(pos_array[0])

            # Set position for all finger joints
            for joint_id in self.finger_joints:
                p.setJointMotorControl2(
                    self.robot_id, joint_id,
                    p.POSITION_CONTROL,
                    targetPosition=pos,
                    force=force,
                    maxVelocity=0.5  # Max velocity 0.5 m/s
                )

            p.stepSimulation()
            time.sleep(self.dt)

            # Check for contact if enabled (when closing)
            if detect_contact and target_pos_per_finger < start_pos:
                contact_forces = self._check_contact_forces()
                if len(contact_forces) > 0:
                    avg_force = np.mean(contact_forces)
                    if avg_force > 5.0:  # 5N threshold for contact detection
                        self.contact_detected = True
                        self.last_contact_force = avg_force
                        self.current_position = pos
                        print(f"Contact detected at {pos*2000:.1f}mm opening "
                              f"(force: {avg_force:.1f}N)")
                        return True  # Object grasped

        self.current_position = target_pos_per_finger
        self.contact_detected = False
        return True

    def _check_contact_forces(self) -> List[float]:
        """
        Check contact forces on gripper fingers.

        Returns:
            List of contact forces (N) detected on finger links
        """
        forces = []

        for joint_id in self.finger_joints:
            # Get contact points for this finger link
            contacts = p.getContactPoints(bodyA=self.robot_id, linkIndexA=joint_id)

            for contact in contacts:
                # contact[9] is normal force
                normal_force = contact[9]
                if normal_force > 0:
                    forces.append(normal_force)

        return forces

    def get_state(self) -> Dict:
        """
        Get current gripper state with detailed information.

        Returns:
            Dictionary with:
                - position_mm: Current opening width in millimeters
                - position_percent: Opening as percentage (0-100)
                - is_open: True if gripper is > 70% open
                - is_closed: True if gripper is < 10% open
                - has_object: True if contact forces detected
                - forces: List of contact forces on each finger
                - joint_positions: Actual joint positions from simulation

        Example:
            >>> state = controller.get_state()
            >>> print(f"Opening: {state['position_mm']:.1f}mm")
            >>> if state['has_object']:
            ...     print(f"Gripping with {state['forces']}N")
        """
        # Read actual joint positions from simulation
        joint_positions = []
        joint_forces = []

        for joint_id in self.finger_joints:
            state = p.getJointState(self.robot_id, joint_id)
            joint_positions.append(state[0])  # Position
            joint_forces.append(state[3])     # Applied force

        # Average position across fingers
        avg_position = np.mean(joint_positions) if joint_positions else self.current_position

        # Convert to millimeters (total opening = 2 * finger position)
        position_mm = avg_position * 2000

        # Calculate percentage
        position_percent = (position_mm / self.GRIPPER_STROKE_MM) * 100

        # Check for object contact
        contact_forces = self._check_contact_forces()
        has_object = len(contact_forces) > 0 and np.mean(contact_forces) > 3.0

        return {
            'position_mm': position_mm,
            'position_percent': position_percent,
            'is_open': position_percent > 70,
            'is_closed': position_percent < 10,
            'has_object': has_object,
            'forces': contact_forces,
            'joint_positions': joint_positions,
            'joint_forces': joint_forces,
        }

    def hold_object(self, hold_force: Optional[float] = None) -> bool:
        """
        Maintain current position with holding force to secure object.

        Args:
            hold_force: Force to maintain (N). If None, uses DEFAULT_HOLD_FORCE

        Returns:
            bool: True if holding force applied successfully
        """
        if hold_force is None:
            hold_force = self.DEFAULT_HOLD_FORCE

        for joint_id in self.finger_joints:
            p.setJointMotorControl2(
                self.robot_id, joint_id,
                p.POSITION_CONTROL,
                targetPosition=self.current_position,
                force=hold_force
            )

        return True

    def release(self, duration: float = 0.5) -> bool:
        """
        Release object by opening gripper with reduced force.

        Args:
            duration: Opening duration in seconds (default: 0.5s)

        Returns:
            bool: True if released successfully
        """
        return self.open(duration=duration, force=self.DEFAULT_OPEN_FORCE)


if __name__ == "__main__":
    # Demo and validation
    print("=" * 60)
    print("DH AG-95 Gripper Controller Demo")
    print("=" * 60)

    # Initialize PyBullet
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)

    # Note: In real usage, this would be initialized with actual robot
    # For demo, we'll just show the API
    print("\nGripper Controller API:")
    print("-" * 60)
    print("Initialization:")
    print("  controller = GripperController(robot_id, finger_joints, dt)")
    print("\nMotion Commands:")
    print("  controller.open(duration=1.0)")
    print("  controller.close(duration=1.0, detect_contact=True)")
    print("  controller.set_position_mm(50)  # 50mm opening")
    print("  controller.set_position_percent(75)  # 75% open")
    print("\nState Query:")
    print("  state = controller.get_state()")
    print("  print(state['position_mm'])  # Current opening in mm")
    print("  print(state['has_object'])   # Object detection")
    print("\nForce Control:")
    print("  controller.hold_object(hold_force=80)")
    print("  controller.release()")

    print("\n" + "=" * 60)
    print("Gripper specifications:")
    print(f"  Model: DH AG-95 Parallel Gripper")
    print(f"  Stroke: {GripperController.GRIPPER_STROKE_MM}mm")
    print(f"  Max Force: {GripperController.MAX_FORCE_PER_FINGER}N per finger")
    print(f"  Resolution: {GripperController.POSITION_RESOLUTION_MM}mm")
    print("=" * 60)

    p.disconnect()
    print("\nDemo complete!")
