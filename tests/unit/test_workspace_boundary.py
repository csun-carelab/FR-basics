"""
Unit tests for WorkspaceBoundary class

Tests workspace boundary validation for FR5 robot.
"""

import pytest
import numpy as np
import sys
import os

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../python_sdk/simulation'))

from workspace_boundaries import WorkspaceBoundary


class TestWorkspaceBoundary:
    """Test suite for WorkspaceBoundary"""

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_valid_joint_positions(self):
        """Valid joint angles should pass"""
        valid_joints = [0, 0, 0, 0, 0, 0]
        result, msg = WorkspaceBoundary.check_joint_limits(valid_joints)

        assert result is True, f"Failed: {msg}"
        assert msg == "OK"

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_invalid_joint_position_j1(self):
        """J1 out of range should fail"""
        invalid_joints = [200, 0, 0, 0, 0, 0]  # J1 limit is ±175°
        result, msg = WorkspaceBoundary.check_joint_limits(invalid_joints)

        assert result is False
        assert "Joint 1" in msg
        assert "out of range" in msg

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_invalid_joint_position_j2(self):
        """J2 out of range should fail"""
        invalid_joints = [0, 100, 0, 0, 0, 0]  # J2 max is +85°
        result, msg = WorkspaceBoundary.check_joint_limits(invalid_joints)

        assert result is False
        assert "Joint 2" in msg

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_edge_case_joint_limits(self):
        """Boundary values should be valid"""
        # Test J1 at max limit
        edge_joints = [175, 0, 0, 0, 0, 0]
        result, msg = WorkspaceBoundary.check_joint_limits(edge_joints)
        assert result is True, f"Max J1 limit should be valid: {msg}"

        # Test J1 at min limit
        edge_joints = [-175, 0, 0, 0, 0, 0]
        result, msg = WorkspaceBoundary.check_joint_limits(edge_joints)
        assert result is True, f"Min J1 limit should be valid: {msg}"

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_wrong_number_of_joints(self):
        """Should fail with wrong number of joints"""
        invalid_joints = [0, 0, 0, 0]  # Only 4 joints
        result, msg = WorkspaceBoundary.check_joint_limits(invalid_joints)

        assert result is False
        assert "Expected 6" in msg

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_joint_clipping(self):
        """Joint clipping should enforce limits"""
        out_of_bounds = [200, 100, 170, 100, 200, 200]
        clipped = WorkspaceBoundary.clip_joint_to_limits(out_of_bounds)

        assert clipped[0] == 175, "J1 should be clipped to 175"
        assert clipped[1] == 85, "J2 should be clipped to 85"
        assert clipped[2] == 162, "J3 should be clipped to 162"

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_valid_cartesian_position(self):
        """Valid TCP position should pass"""
        valid_pos = [0.5, 0.0, 0.5]  # 0.5m forward, 0.5m up
        result, msg = WorkspaceBoundary.check_cartesian_position(valid_pos)

        assert result is True, f"Failed: {msg}"
        assert msg == "OK"

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_position_too_far(self):
        """Position beyond reach should fail"""
        far_pos = [1.0, 0.0, 0.5]  # Beyond 0.85m radius
        result, msg = WorkspaceBoundary.check_cartesian_position(far_pos)

        assert result is False
        assert "out of bounds" in msg.lower()

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_position_too_close(self):
        """Position too close to base should fail"""
        close_pos = [0.05, 0.05, 0.3]  # < 0.15m radius
        result, msg = WorkspaceBoundary.check_cartesian_position(close_pos)

        assert result is False
        assert "too close" in msg.lower()

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_position_negative_z(self):
        """Negative Z (below base) should fail"""
        below_base = [0.5, 0.0, -0.1]
        result, msg = WorkspaceBoundary.check_cartesian_position(below_base)

        assert result is False
        assert "Z position" in msg

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_position_too_high(self):
        """Position above max Z should fail"""
        too_high = [0.5, 0.0, 1.5]  # Above 1.2m
        result, msg = WorkspaceBoundary.check_cartesian_position(too_high)

        assert result is False
        assert "Z position" in msg

    @pytest.mark.unit
    @pytest.mark.workspace
    def test_workspace_info_string(self):
        """Workspace info should return formatted string"""
        info = WorkspaceBoundary.get_workspace_info()

        assert isinstance(info, str)
        assert "FR5 Workspace Boundaries" in info
        assert "Joint Limits" in info
        assert "Cartesian Workspace" in info


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
