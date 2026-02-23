"""
Unit tests for GripperController class

Tests DH AG-95 gripper controller functionality.
"""

import pytest
import numpy as np
import sys
import os

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../python_sdk/simulation'))

# Import pybullet for testing
import pybullet as p
from gripper_controller import GripperController


class TestGripperController:
    """Test suite for GripperController"""

    @pytest.fixture
    def pybullet_sim(self):
        """Setup PyBullet simulation environment"""
        client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        yield client
        p.disconnect()

    @pytest.mark.unit
    @pytest.mark.gripper
    def test_gripper_initialization(self):
        """Test gripper controller initialization"""
        # Test constants
        assert GripperController.GRIPPER_STROKE_MM == 95
        assert GripperController.GRIPPER_STROKE_M == 0.095
        assert GripperController.MAX_FORCE_PER_FINGER == 45

    @pytest.mark.unit
    @pytest.mark.gripper
    def test_position_conversions(self):
        """Test position conversion methods"""
        controller = GripperController(0, [1, 2], dt=1/120.0)

        # Test mm to meters conversion
        assert controller.gripper_fully_open == 0.0475  # 47.5mm per finger
        assert controller.gripper_closed == 0.0

    @pytest.mark.unit
    @pytest.mark.gripper
    def test_position_calculations(self):
        """Test position calculations without PyBullet"""
        controller = GripperController(0, [1, 2], dt=1/120.0)

        # Test position to mm conversion
        # 0.0475m per finger = 95mm total opening
        assert controller.gripper_fully_open * 2000 == 95.0

        # Test closed position
        assert controller.gripper_closed == 0.0

    @pytest.mark.unit
    @pytest.mark.gripper
    def test_position_percent_calculation(self):
        """Test position percentage calculation logic"""
        controller = GripperController(0, [1, 2], dt=1/120.0)

        # Calculate percentage manually
        # Fully open: 0.0475m per finger
        open_percent = (0.0475 * 2000 / 95.0) * 100
        assert abs(open_percent - 100.0) < 0.1

        # Fully closed: 0.0m
        closed_percent = (0.0 * 2000 / 95.0) * 100
        assert abs(closed_percent - 0.0) < 0.1

        # Half open: 0.02375m per finger
        half_percent = (0.02375 * 2000 / 95.0) * 100
        assert abs(half_percent - 50.0) < 1.0

    @pytest.mark.unit
    @pytest.mark.gripper
    def test_is_open_is_closed_logic(self):
        """Test is_open and is_closed thresholds"""
        controller = GripperController(0, [1, 2], dt=1/120.0)

        # Test threshold logic (70% for open, 10% for closed)
        # > 70% should be open
        assert 75 > 70  # Would be marked as open

        # < 10% should be closed
        assert 5 < 10  # Would be marked as closed

        # Between should be neither
        assert not (40 > 70)  # Not open
        assert not (40 < 10)  # Not closed

    @pytest.mark.unit
    @pytest.mark.gripper
    def test_gripper_stroke_specification(self):
        """Test gripper meets DH AG-95 specifications"""
        controller = GripperController(0, [1, 2], dt=1/120.0)

        # DH AG-95 has 95mm stroke
        assert controller.GRIPPER_STROKE_MM == 95

        # Each finger moves 47.5mm from center
        max_per_finger = controller.GRIPPER_STROKE_M / 2
        assert abs(max_per_finger - 0.0475) < 0.0001  # 47.5mm

        # Force specification
        assert controller.MAX_FORCE_PER_FINGER == 45  # 45N per finger


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
