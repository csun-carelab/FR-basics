# Python SDK for FR5 Cobot

This folder contains the plain Python SDK integration for the Fairino FR5 collaborative robot. It provides direct access to robot control without ROS2 dependencies.

## Modes

- **real_robot/**: Control the physical FR5 robot using the Fairino SDK.
- **simulation/**: Simulate robot behavior using PyBullet for testing and development.

## Installation

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. For simulation mode, ensure PyBullet GUI works on your system.

## Usage

Choose your mode:

- **Real Robot**: Navigate to `real_robot/` and run `python first_example.py`
- **Simulation**: Navigate to `simulation/` and run `python first_example.py`

Both modes perform the same actions: connect, read joint positions, move first joint by +15 degrees, and close.

## Requirements

- Python 3.8+
- Fairino SDK (for real robot mode)
- PyBullet (for simulation mode)