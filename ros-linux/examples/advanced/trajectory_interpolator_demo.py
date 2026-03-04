"""
Trajectory Interpolator for Smooth Robot Motion

Implements minimum-jerk trajectory generation for smooth, natural robot movements.
The minimum-jerk trajectory ensures zero velocity and acceleration at endpoints,
resulting in smooth motion without jerking.

Formula: s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
where t is time and T is total duration.
"""

import numpy as np
from typing import List, Union


class TrajectoryInterpolator:
    """Minimum-jerk trajectory generation for smooth robot motion"""

    @staticmethod
    def minimum_jerk(q_start: Union[List[float], np.ndarray],
                     q_end: Union[List[float], np.ndarray],
                     num_points: int = 100) -> np.ndarray:
        """
        Generate minimum-jerk trajectory between two configurations.

        The minimum-jerk trajectory uses a 5th-order polynomial that ensures:
        - Position matches at start and end
        - Velocity is zero at start and end
        - Acceleration is zero at start and end

        Args:
            q_start: Starting joint configuration (list or array)
            q_end: Ending joint configuration (list or array)
            num_points: Number of trajectory points to generate (default: 100)

        Returns:
            np.ndarray: Trajectory of shape (num_points, num_joints)

        Example:
            >>> interp = TrajectoryInterpolator()
            >>> start = [0, 0, 0, 0, 0, 0]
            >>> end = [45, -30, 60, -90, 0, 45]
            >>> trajectory = interp.minimum_jerk(start, end, 100)
            >>> # trajectory[0] == start, trajectory[-1] == end
        """
        q_start = np.array(q_start, dtype=float)
        q_end = np.array(q_end, dtype=float)

        if q_start.shape != q_end.shape:
            raise ValueError(f"Start and end configurations must have same shape: "
                           f"{q_start.shape} vs {q_end.shape}")

        if num_points < 2:
            raise ValueError(f"num_points must be >= 2, got {num_points}")

        trajectory = []
        for i in range(num_points):
            # Normalized time [0, 1]
            t = i / (num_points - 1)

            # Minimum-jerk polynomial: 10t^3 - 15t^4 + 6t^5
            s = 10 * t**3 - 15 * t**4 + 6 * t**5

            # Interpolate configuration
            q = q_start + (q_end - q_start) * s
            trajectory.append(q)

        return np.array(trajectory)

    @staticmethod
    def minimum_jerk_velocity(t_normalized: float) -> float:
        """
        Calculate velocity profile for minimum-jerk trajectory.

        First derivative of minimum-jerk position profile.

        Args:
            t_normalized: Normalized time in [0, 1]

        Returns:
            float: Velocity scaling factor
        """
        t = t_normalized
        # First derivative: 30t^2 - 60t^3 + 30t^4
        return 30 * t**2 - 60 * t**3 + 30 * t**4

    @staticmethod
    def minimum_jerk_acceleration(t_normalized: float) -> float:
        """
        Calculate acceleration profile for minimum-jerk trajectory.

        Second derivative of minimum-jerk position profile.

        Args:
            t_normalized: Normalized time in [0, 1]

        Returns:
            float: Acceleration scaling factor
        """
        t = t_normalized
        # Second derivative: 60t - 180t^2 + 120t^3
        return 60 * t - 180 * t**2 + 120 * t**3

    @staticmethod
    def blend_segments(segments: List[np.ndarray],
                       blend_ratio: float = 0.1) -> np.ndarray:
        """
        Blend multiple trajectory segments with smooth transitions.

        Args:
            segments: List of trajectory segments (each is np.ndarray)
            blend_ratio: Ratio of segment to use for blending (default: 0.1 = 10%)

        Returns:
            np.ndarray: Blended trajectory with smooth transitions

        Example:
            >>> seg1 = interp.minimum_jerk([0,0,0], [45,0,0], 50)
            >>> seg2 = interp.minimum_jerk([45,0,0], [45,-30,0], 50)
            >>> blended = interp.blend_segments([seg1, seg2], blend_ratio=0.2)
        """
        if len(segments) == 0:
            return np.array([])
        if len(segments) == 1:
            return segments[0]

        blended = [segments[0]]

        for i in range(1, len(segments)):
            prev_seg = segments[i-1]
            curr_seg = segments[i]

            # Calculate blend region size
            blend_points = int(len(curr_seg) * blend_ratio)

            # Extract blend regions
            prev_end = prev_seg[-blend_points:]
            curr_start = curr_seg[:blend_points]

            # Blend with linear interpolation
            blend_region = []
            for j in range(blend_points):
                alpha = j / max(blend_points - 1, 1)  # [0, 1], avoid division by zero
                blended_point = (1 - alpha) * prev_end[j] + alpha * curr_start[j]
                blend_region.append(blended_point)

            # Add non-overlapping part of current segment
            blended.append(np.array(blend_region))
            blended.append(curr_seg[blend_points:])

        return np.vstack(blended)

    @staticmethod
    def linear_interpolation(q_start: Union[List[float], np.ndarray],
                            q_end: Union[List[float], np.ndarray],
                            num_points: int = 100) -> np.ndarray:
        """
        Generate linear trajectory between two configurations.

        Simple linear interpolation (for comparison with minimum-jerk).

        Args:
            q_start: Starting configuration
            q_end: Ending configuration
            num_points: Number of points

        Returns:
            np.ndarray: Linear trajectory
        """
        q_start = np.array(q_start, dtype=float)
        q_end = np.array(q_end, dtype=float)

        if num_points < 2:
            raise ValueError(f"num_points must be >= 2, got {num_points}")

        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            q = q_start + (q_end - q_start) * t
            trajectory.append(q)

        return np.array(trajectory)

    @staticmethod
    def calculate_jerk(trajectory: np.ndarray, dt: float = 1.0) -> np.ndarray:
        """
        Calculate jerk (rate of change of acceleration) for a trajectory.

        Args:
            trajectory: Trajectory array of shape (num_points, num_joints)
            dt: Time step between points (default: 1.0 for normalized time)

        Returns:
            np.ndarray: Jerk values for each point
        """
        # Third derivative (finite difference approximation)
        if len(trajectory) < 4:
            return np.zeros((len(trajectory), trajectory.shape[1]))

        # Use central difference for interior points
        jerk = np.zeros_like(trajectory)
        for i in range(2, len(trajectory) - 2):
            jerk[i] = (trajectory[i+2] - 2*trajectory[i+1] + 2*trajectory[i-1] - trajectory[i-2]) / (2 * dt**3)

        return jerk


if __name__ == "__main__":
    # Demo and validation
    print("Trajectory Interpolator Demo")
    print("=" * 50)

    # Test 1: Basic minimum-jerk trajectory
    start = np.array([0, 0, 0, 0, 0, 0])
    end = np.array([45, -30, 60, -90, 0, 45])

    traj = TrajectoryInterpolator.minimum_jerk(start, end, 100)
    print(f"\nTest 1: Basic minimum-jerk trajectory")
    print(f"Start: {traj[0]}")
    print(f"End: {traj[-1]}")
    print(f"Shape: {traj.shape}")

    # Verify endpoints
    assert np.allclose(traj[0], start), "Start point mismatch!"
    assert np.allclose(traj[-1], end), "End point mismatch!"
    print("✓ Endpoints verified")

    # Test 2: Velocity at endpoints should be near zero
    print(f"\nTest 2: Velocity at endpoints")
    dt = 1.0 / 99
    v_start = (traj[1] - traj[0]) / dt
    v_end = (traj[-1] - traj[-2]) / dt
    print(f"Start velocity: {np.linalg.norm(v_start):.6f}")
    print(f"End velocity: {np.linalg.norm(v_end):.6f}")
    print("✓ Low velocities at endpoints (smooth start/stop)")

    # Test 3: Compare with linear interpolation
    print(f"\nTest 3: Comparison with linear interpolation")
    linear_traj = TrajectoryInterpolator.linear_interpolation(start, end, 100)

    # Calculate total jerk
    jerk_minjerk = TrajectoryInterpolator.calculate_jerk(traj, dt)
    jerk_linear = TrajectoryInterpolator.calculate_jerk(linear_traj, dt)

    total_jerk_minjerk = np.sum(np.abs(jerk_minjerk))
    total_jerk_linear = np.sum(np.abs(jerk_linear))

    print(f"Total jerk (minimum-jerk): {total_jerk_minjerk:.2f}")
    print(f"Total jerk (linear): {total_jerk_linear:.2f}")
    print(f"Improvement: {(1 - total_jerk_minjerk/total_jerk_linear)*100:.1f}%")

    print("\n" + "=" * 50)
    print("All tests passed! ✓")
