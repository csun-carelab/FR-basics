# Installation Instructions

This document provides detailed installation instructions for both Python SDK and ROS2 SDK modes.

## Prerequisites

- Ubuntu 22.04 LTS or later
- Python 3.8 or later
- Git

## Python SDK Installation

### Real Robot Mode

1. Install Fairino SDK:
   ```bash
   pip install fairino
   ```

2. Clone and setup:
   ```bash
   git clone https://github.com/csun-carelab/FR-basics.git
   cd FR-basics/python_sdk
   pip install -r requirements.txt
   ```

3. Configure robot IP (default: 192.168.58.2) in `real_robot/first_example.py` if needed.

### Simulation Mode

1. Install PyBullet:
   ```bash
   pip install pybullet numpy
   ```

2. Setup:
   ```bash
   cd FR-basics/python_sdk/simulation
   # URDF files are included
   ```

## ROS2 SDK Installation

### Install ROS2

1. Follow official ROS2 installation: https://docs.ros.org/en/jazzy/Installation.html

2. For Ubuntu 22.04 with Jazzy:
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu/. ./jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt upgrade
   sudo apt install ros-jazzy-desktop
   ```

3. Source ROS2:
   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Build FR5 ROS2 Workspace

1. Install dependencies:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

2. Build:
   ```bash
   cd FR-basics/ros2_sdk
   colcon build --symlink-install
   echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
   source install/setup.bash
   ```

## Verification

### Python SDK
```bash
cd python_sdk/simulation
python first_example.py  # Should open PyBullet GUI
```

### ROS2 SDK
```bash
cd ros2_sdk
ros2 pkg list | grep fr5  # Should show FR5 packages
```

## Troubleshooting

- **PyBullet GUI not opening**: Ensure X11/display is available
- **ROS2 build fails**: Check all dependencies installed
- **Robot connection fails**: Verify IP and network