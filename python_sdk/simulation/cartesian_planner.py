"""
Cartesian Path Planning for FR5 Robot

Provides Cartesian space motion planning (straight-line motion in workspace).
Implements linear interpolation in Cartesian space with inverse kinematics.
"""

import numpy as np
from typing import List, Tuple, Optional
import pybullet as p


class CartesianPlanner:
    """
    Cartesian space path planner for FR5 robot.

    Generates straight-line paths in Cartesian space by interpolating
    end-effector poses and solving inverse kinematics.
    """

    @staticmethod
    def interpolate_pose(pose_start: List[float], pose_end: List[float],
                        num_points: int = 50) -> List[List[float]]:
        """
        Interpolate linearly between two Cartesian poses.

        Args:
            pose_start: Starting pose [x, y, z, rx, ry, rz] in meters and radians
            pose_end: Ending pose [x, y, z, rx, ry, rz]
            num_points: Number of interpolation points (default: 50)

        Returns:
            List of interpolated poses

        Example:
            >>> start = [0.5, 0.0, 0.4, 3.14, 0, 0]
            >>> end = [0.5, 0.2, 0.4, 3.14, 0, 0]
            >>> path = CartesianPlanner.interpolate_pose(start, end, 20)
        """
        pose_start = np.array(pose_start)
        pose_end = np.array(pose_end)

        if num_points < 2:
            raise ValueError(f"num_points must be >= 2, got {num_points}")

        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            pose = pose_start + (pose_end - pose_start) * t
            path.append(pose.tolist())

        return path

    @staticmethod
    def plan_line(robot_id: int, ee_link_index: int,
                  target_pose: List[float], current_joints: List[float],
                  num_waypoints: int = 30) -> Optional[List[np.ndarray]]:
        """
        Plan straight-line Cartesian path to target pose.

        Args:
            robot_id: PyBullet robot body ID
            ee_link_index: End-effector link index
            target_pose: Target pose [x, y, z, rx, ry, rz]
            current_joints: Current joint configuration
            num_waypoints: Number of waypoints along path

        Returns:
            List of joint configurations, or None if IK fails

        Example:
            >>> path = CartesianPlanner.plan_line(
            ...     robot_id, ee_link, [0.5, 0.2, 0.4, 3.14, 0, 0],
            ...     current_joints, num_waypoints=20
            ... )
        """
        # Get current end-effector pose
        link_state = p.getLinkState(robot_id, ee_link_index)
        current_pos = link_state[0]
        current_orn_quat = link_state[1]
        current_orn = p.getEulerFromQuaternion(current_orn_quat)

        current_pose = list(current_pos) + list(current_orn)

        # Interpolate Cartesian path
        cartesian_path = CartesianPlanner.interpolate_pose(
            current_pose, target_pose, num_waypoints
        )

        # Solve IK for each Cartesian waypoint
        joint_path = []
        prev_joints = np.array(current_joints)

        for pose in cartesian_path:
            pos = pose[:3]
            orn = pose[3:]
            orn_quat = p.getQuaternionFromEuler(orn)

            # Calculate IK
            ik_joints = p.calculateInverseKinematics(
                robot_id,
                ee_link_index,
                pos,
                orn_quat,
                maxNumIterations=100,
                residualThreshold=0.001
            )

            # Take first 6 joints (arm only)
            joints = np.array(ik_joints[:6])

            # Verify IK solution by checking end-effector position error
            for j_idx, j_val in enumerate(joints):
                p.resetJointState(robot_id, j_idx, j_val)
            actual_pos = p.getLinkState(robot_id, ee_link_index)[0]
            position_error = np.linalg.norm(np.array(actual_pos) - np.array(pos))
            if position_error > 0.01:  # 1cm tolerance
                print(f"IK failed at waypoint: position error {position_error:.4f}m")
                return None

            # Check for large jumps (singularities)
            joint_diff = np.abs(joints - prev_joints)
            if np.any(joint_diff > np.pi):
                print(f"Warning: Large joint jump detected")

            joint_path.append(joints)
            prev_joints = joints

        return joint_path

    @staticmethod
    def validate_cartesian_path(path: List[np.ndarray],
                               max_joint_diff: float = 0.5) -> bool:
        """
        Validate Cartesian path for smoothness and feasibility.

        Args:
            path: List of joint configurations
            max_joint_diff: Maximum allowed joint difference between waypoints (radians)

        Returns:
            True if path is valid, False otherwise
        """
        if path is None or len(path) < 2:
            return False

        for i in range(len(path) - 1):
            diff = np.abs(path[i+1] - path[i])
            if np.any(diff > max_joint_diff):
                return False

        return True


class TrajectoryRecorder:
    """
    Records and plays back robot trajectories.

    Useful for teaching by demonstration and trajectory optimization.
    """

    def __init__(self):
        """Initialize trajectory recorder"""
        self.recorded_trajectories = {}
        self.current_recording = []
        self.is_recording = False

    def start_recording(self, name: str):
        """
        Start recording a trajectory.

        Args:
            name: Name for this trajectory
        """
        self.current_recording = []
        self.is_recording = True
        print(f"Started recording trajectory: {name}")

    def record_waypoint(self, joint_positions: List[float], timestamp: float = None):
        """
        Record a waypoint during trajectory recording.

        Args:
            joint_positions: Current joint positions
            timestamp: Optional timestamp (default: sequential)
        """
        if not self.is_recording:
            return

        waypoint = {
            'joints': np.array(joint_positions),
            'timestamp': timestamp if timestamp is not None else len(self.current_recording)
        }
        self.current_recording.append(waypoint)

    def stop_recording(self, name: str) -> int:
        """
        Stop recording and save trajectory.

        Args:
            name: Name to save trajectory under

        Returns:
            Number of waypoints recorded
        """
        if not self.is_recording:
            return 0

        self.recorded_trajectories[name] = self.current_recording.copy()
        self.is_recording = False

        num_waypoints = len(self.current_recording)
        print(f"Stopped recording '{name}': {num_waypoints} waypoints")

        return num_waypoints

    def get_trajectory(self, name: str) -> Optional[List[np.ndarray]]:
        """
        Get recorded trajectory by name.

        Args:
            name: Trajectory name

        Returns:
            List of joint configurations, or None if not found
        """
        if name not in self.recorded_trajectories:
            return None

        return [wp['joints'] for wp in self.recorded_trajectories[name]]

    def save_trajectory(self, name: str, filename: str):
        """
        Save trajectory to file.

        Args:
            name: Trajectory name
            filename: Output filename (.npy format)
        """
        if name not in self.recorded_trajectories:
            print(f"Trajectory '{name}' not found")
            return

        trajectory_data = {
            'name': name,
            'waypoints': [wp['joints'] for wp in self.recorded_trajectories[name]],
            'timestamps': [wp['timestamp'] for wp in self.recorded_trajectories[name]]
        }

        np.save(filename, trajectory_data)
        print(f"Saved trajectory '{name}' to {filename}")

    def load_trajectory(self, filename: str) -> str:
        """
        Load trajectory from file.

        Args:
            filename: Input filename (.npy format)

        Returns:
            Name of loaded trajectory
        """
        trajectory_data = np.load(filename, allow_pickle=True).item()

        name = trajectory_data['name']
        waypoints = trajectory_data['waypoints']
        timestamps = trajectory_data['timestamps']

        self.recorded_trajectories[name] = [
            {'joints': wp, 'timestamp': ts}
            for wp, ts in zip(waypoints, timestamps)
        ]

        print(f"Loaded trajectory '{name}' with {len(waypoints)} waypoints")
        return name

    def list_trajectories(self) -> List[str]:
        """
        List all recorded trajectory names.

        Returns:
            List of trajectory names
        """
        return list(self.recorded_trajectories.keys())

    def clear_trajectory(self, name: str):
        """Delete a recorded trajectory"""
        if name in self.recorded_trajectories:
            del self.recorded_trajectories[name]
            print(f"Deleted trajectory '{name}'")

    def clear_all(self):
        """Delete all recorded trajectories"""
        self.recorded_trajectories.clear()
        print("Cleared all trajectories")


if __name__ == "__main__":
    print("=" * 70)
    print("Cartesian Planner & Trajectory Recorder Demo")
    print("=" * 70)

    print("\n1. Cartesian Path Planning:")
    print("-" * 70)
    print("from cartesian_planner import CartesianPlanner")
    print("")
    print("# Interpolate Cartesian path")
    print("start_pose = [0.5, 0.0, 0.4, 3.14, 0, 0]")
    print("end_pose = [0.5, 0.2, 0.4, 3.14, 0, 0]")
    print("cart_path = CartesianPlanner.interpolate_pose(start_pose, end_pose, 30)")
    print("")
    print("# Plan with IK")
    print("joint_path = CartesianPlanner.plan_line(")
    print("    robot_id, ee_link, end_pose, current_joints, num_waypoints=30")
    print(")")

    print("\n2. Trajectory Recording:")
    print("-" * 70)
    print("from cartesian_planner import TrajectoryRecorder")
    print("")
    print("recorder = TrajectoryRecorder()")
    print("")
    print("# Start recording")
    print("recorder.start_recording('pick_and_place_1')")
    print("")
    print("# Record waypoints during motion")
    print("for _ in range(100):")
    print("    joints = robot.get_joint_positions()")
    print("    recorder.record_waypoint(joints)")
    print("")
    print("# Stop and save")
    print("recorder.stop_recording('pick_and_place_1')")
    print("recorder.save_trajectory('pick_and_place_1', 'trajectory.npy')")
    print("")
    print("# Load and replay")
    print("recorder.load_trajectory('trajectory.npy')")
    print("path = recorder.get_trajectory('pick_and_place_1')")

    print("\n" + "=" * 70)
    print("Features:")
    print("  ✓ Cartesian space interpolation")
    print("  ✓ IK-based path planning")
    print("  ✓ Trajectory recording and playback")
    print("  ✓ Save/load trajectories to disk")
    print("  ✓ Path validation")
    print("=" * 70)
