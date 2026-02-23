# ROS2 SDK for FR5 Cobot

This folder contains the ROS2-based integration for the Fairino FR5 collaborative robot. It provides advanced robot control using ROS2 middleware.

## Prerequisites

- Ubuntu 22.04 or later
- ROS2 Jazzy or Humble
- Fairino SDK (for real robot mode)

## Installation

1. Install ROS2: Follow https://docs.ros.org/en/jazzy/Installation.html

2. Build the workspace:
   ```bash
   cd ros2_sdk
   colcon build --symlink-install
   source install/setup.bash
   ```

## Modes

- **real_robot/**: Control the physical FR5 robot via ROS2 nodes and services.
- **simulation/**: Simulate robot behavior using ROS2 with PyBullet.

## Usage

Choose your mode:

- **Real Robot**: Navigate to `real_robot/` and run `python first_example.py`
- **Simulation**: Navigate to `simulation/` and run `python first_example.py`

Both modes perform the same actions: launch ROS2 nodes, read joint positions, move first joint by +15 degrees, and shutdown.

## ROS2 Services

- `/fr5/set_joint_position`: Set joint positions
- `/fr5/get_robot_state`: Get current robot state
- `/joint_states`: Joint state topic

## Requirements

- ROS2 Jazzy/Humble
- Colcon build tools
- Fairino SDK (real robot only)