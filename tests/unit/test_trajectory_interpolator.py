"""
Unit tests for TrajectoryInterpolator class

Tests minimum-jerk trajectory generation for smooth robot motion.
"""

import pytest
import numpy as np
import sys
import os

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../python_sdk/simulation'))

from trajectory_interpolator import TrajectoryInterpolator


class TestTrajectoryInterpolator:
    """Test suite for TrajectoryInterpolator"""

    @pytest.mark.unit
    def test_minimum_jerk_endpoints(self):
        """Trajectory should start and end at correct positions"""
        q_start = np.array([0, 0, 0, 0, 0, 0])
        q_end = np.array([45, -30, 60, -90, 0, 45])

        trajectory = TrajectoryInterpolator.minimum_jerk(q_start, q_end, 100)

        # Check endpoints
        np.testing.assert_array_almost_equal(trajectory[0], q_start, decimal=6)
        np.testing.assert_array_almost_equal(trajectory[-1], q_end, decimal=6)

    @pytest.mark.unit
    def test_minimum_jerk_shape(self):
        """Trajectory should have correct shape"""
        q_start = np.array([0, 0, 0])
        q_end = np.array([1, 2, 3])
        num_points = 50

        trajectory = TrajectoryInterpolator.minimum_jerk(q_start, q_end, num_points)

        assert trajectory.shape == (num_points, 3), f"Expected shape ({num_points}, 3), got {trajectory.shape}"

    @pytest.mark.unit
    def test_minimum_jerk_monotonic(self):
        """Trajectory should be monotonic for monotonic endpoints"""
        q_start = np.array([0.0])
        q_end = np.array([1.0])

        trajectory = TrajectoryInterpolator.minimum_jerk(q_start, q_end, 100)

        # Check monotonicity
        for i in range(1, len(trajectory)):
            assert trajectory[i][0] >= trajectory[i-1][0], \
                f"Non-monotonic at step {i}: {trajectory[i-1][0]} -> {trajectory[i][0]}"

    @pytest.mark.unit
    def test_minimum_jerk_zero_velocity_endpoints(self):
        """Velocity should be near zero at start and end (smooth start/stop)"""
        q_start = np.array([0.0])
        q_end = np.array([1.0])

        trajectory = TrajectoryInterpolator.minimum_jerk(q_start, q_end, 200)

        # Numerical derivative at endpoints
        dt = 1.0 / 199
        v_start = (trajectory[1] - trajectory[0]) / dt
        v_end = (trajectory[-1] - trajectory[-2]) / dt

        # Start/end velocities should be small for minimum-jerk
        assert np.abs(v_start[0]) < 0.15, f"Start velocity too high: {v_start[0]}"
        assert np.abs(v_end[0]) < 0.15, f"End velocity too high: {v_end[0]}"

    @pytest.mark.unit
    def test_linear_interpolation(self):
        """Linear interpolation should produce straight line"""
        q_start = np.array([0.0, 0.0])
        q_end = np.array([1.0, 2.0])

        trajectory = TrajectoryInterpolator.linear_interpolation(q_start, q_end, 11)

        # Check specific points
        np.testing.assert_array_almost_equal(trajectory[0], [0.0, 0.0])
        np.testing.assert_array_almost_equal(trajectory[5], [0.5, 1.0])  # Midpoint
        np.testing.assert_array_almost_equal(trajectory[10], [1.0, 2.0])

    @pytest.mark.unit
    def test_mismatched_dimensions(self):
        """Should raise error for mismatched start/end dimensions"""
        q_start = np.array([0, 0, 0])
        q_end = np.array([1, 1])  # Wrong dimension

        with pytest.raises(ValueError, match="same shape"):
            TrajectoryInterpolator.minimum_jerk(q_start, q_end, 100)

    @pytest.mark.unit
    @pytest.mark.slow
    def test_large_trajectory(self):
        """Should handle large number of points"""
        q_start = np.array([0.0] * 6)
        q_end = np.array([90.0] * 6)

        trajectory = TrajectoryInterpolator.minimum_jerk(q_start, q_end, 10000)

        assert len(trajectory) == 10000
        np.testing.assert_array_almost_equal(trajectory[0], q_start)
        np.testing.assert_array_almost_equal(trajectory[-1], q_end)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
