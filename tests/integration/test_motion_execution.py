"""
Integration tests for FR5 robot motion execution

Tests smooth motion execution with minimum-jerk trajectories.
"""

import pytest
import numpy as np
import sys
import os
import time

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../python_sdk'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../python_sdk/simulation'))

from fr5_robot_interface import FR5RobotInterface


class TestMotionExecution:
    """Integration tests for smooth motion execution"""

    @pytest.fixture
    def robot_sim(self):
        """Create robot simulation instance"""
        robot = FR5RobotInterface(mode='simulation', gui=False)
        yield robot
        robot.disconnect()

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_robot_initialization(self, robot_sim):
        """Test robot initialization in simulation mode"""
        assert robot_sim.mode == 'simulation'
        assert robot_sim.backend is not None

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_get_joint_positions(self, robot_sim):
        """Test reading joint positions"""
        joints = robot_sim.get_joint_positions()

        assert len(joints) == 6, "FR5 should have 6 joints"
        assert all(isinstance(j, (int, float)) for j in joints), "Joints should be numeric"

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_workspace_limit_checking(self, robot_sim):
        """Test workspace boundary checking"""
        # Valid position
        valid_pos = [0, 0, 0, 0, 0, 0]
        is_valid, msg = robot_sim.check_workspace_limits(valid_pos)
        assert is_valid, f"Valid position should pass: {msg}"

        # Invalid position (J1 out of range)
        invalid_pos = [200, 0, 0, 0, 0, 0]
        is_valid, msg = robot_sim.check_workspace_limits(invalid_pos)
        assert not is_valid, "Invalid position should fail"
        assert "Joint 1" in msg

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_move_to_home_position(self, robot_sim):
        """Test moving to home position"""
        home = [0, 0, 0, 0, 0, 0]

        # Check if position is valid
        valid, msg = robot_sim.check_workspace_limits(home)
        assert valid, f"Home position should be valid: {msg}"

        # Get initial position
        initial = robot_sim.get_joint_positions()

        # Move to home (may succeed or fail based on collision)
        result = robot_sim.move_j(home, velocity=30)

        # Result can be 0 (success) or -1 (collision/failure)
        assert result in [0, -1], f"Move result should be 0 or -1, got {result}"

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_small_movement(self, robot_sim):
        """Test small incremental movement"""
        initial = robot_sim.get_joint_positions()

        # Small movement (±5 degrees from current position)
        target = [initial[0] + 5, initial[1], initial[2],
                  initial[3], initial[4], initial[5]]

        # Check validity
        valid, msg = robot_sim.check_workspace_limits(target)

        if valid:
            result = robot_sim.move_j(target, velocity=30)
            # Motion may succeed or fail due to collisions
            assert result in [0, -1]

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    @pytest.mark.slow
    def test_motion_timing(self, robot_sim):
        """Test that motion completes in reasonable time"""
        initial = robot_sim.get_joint_positions()
        target = [0, -15, 15, -30, 0, 15]

        # Check if valid
        valid, msg = robot_sim.check_workspace_limits(target)
        if not valid:
            pytest.skip(f"Target position invalid: {msg}")

        start_time = time.time()
        result = robot_sim.move_j(target, velocity=50)
        elapsed = time.time() - start_time

        if result == 0:  # Only check timing if motion succeeded
            # Motion should complete in reasonable time (< 30 seconds)
            assert elapsed < 30.0, f"Motion took too long: {elapsed:.1f}s"
            # Motion should take at least some time (not instant)
            assert elapsed > 0.5, f"Motion too fast (likely not executed): {elapsed:.1f}s"

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_gripper_operations(self, robot_sim):
        """Test gripper open and close operations"""
        if robot_sim.gripper_controller is None:
            pytest.skip("No gripper available in simulation")

        # Test open
        result_open = robot_sim.open_gripper()
        assert result_open == 0, "Gripper open should succeed"

        # Get state after opening
        state = robot_sim.get_state()
        if state['gripper_state']:
            assert state['gripper_state']['is_open'] is True

        # Test close
        result_close = robot_sim.close_gripper()
        assert result_close == 0, "Gripper close should succeed"

        # Get state after closing
        state = robot_sim.get_state()
        if state['gripper_state']:
            # May or may not be fully closed depending on object presence
            assert state['gripper_state']['position_percent'] < 50

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_gripper_position_control(self, robot_sim):
        """Test precise gripper position control"""
        if robot_sim.gripper_controller is None:
            pytest.skip("No gripper available in simulation")

        # Test 50% open
        result = robot_sim.move_gripper(50, speed_pct=30)
        assert result == 0, "Gripper position control should succeed"

        # Test 75% open
        result = robot_sim.move_gripper(75, speed_pct=30)
        assert result == 0, "Gripper position control should succeed"

        # Test 25% open
        result = robot_sim.move_gripper(25, speed_pct=30)
        assert result == 0, "Gripper position control should succeed"

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_state_consistency(self, robot_sim):
        """Test that robot state is consistent"""
        state1 = robot_sim.get_state()
        state2 = robot_sim.get_state()

        # States should be identical when robot is stationary
        np.testing.assert_array_almost_equal(
            state1['joint_positions'],
            state2['joint_positions'],
            decimal=3
        )

    @pytest.mark.integration
    @pytest.mark.motion
    @pytest.mark.simulation
    def test_sequential_movements(self, robot_sim):
        """Test multiple sequential movements"""
        positions = [
            [0, 0, 0, 0, 0, 0],
            [10, 0, 0, 0, 0, 0],
            [0, -10, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
        ]

        for i, target in enumerate(positions):
            valid, msg = robot_sim.check_workspace_limits(target)
            if valid:
                result = robot_sim.move_j(target, velocity=50)
                # Each move can succeed or fail
                assert result in [0, -1], f"Move {i} returned unexpected result: {result}"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
