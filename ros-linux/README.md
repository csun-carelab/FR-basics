# ros-linux

ROS2 add-ons and examples for the Fairino FR5 on Ubuntu/Linux.

## Prerequisites

### Required

- **Ubuntu 22.04** (or compatible)
- **ROS2 Humble** (or newer), sourced (`source /opt/ros/humble/setup.bash`)
- **Python packages:**
  ```bash
  pip install pybullet numpy
  ```
- **Fairino ROS2 driver** and `fairino_msgs` — installed per Fairino documentation

### Optional

- **MoveIt2** — for motion planning demos:
  ```bash
  sudo apt install ros-humble-moveit
  ```

## Structure

```
ros-linux/
├── ros2-sdk/                      Add-on Python modules and packages
│   ├── rrt_planner.py             RRT-Connect planner (rclpy-aware)
│   ├── trajectory_interpolator.py Min-jerk interpolation
│   ├── workspace_boundaries.py    Gripper-aware workspace limits
│   ├── fr5_pybullet_sim/          ROS2 package: PyBullet simulation node
│   ├── dh_gripper/                AG-95 gripper URDF and meshes
│   └── urdf/                      Standalone FR5 URDFs
│
├── examples/
│   ├── basic/                     4 beginner examples
│   └── advanced/                  MoveIt2 and RRT demos
│
└── config/
    └── fairino5_v6_moveit2_config/ MoveIt2 configuration package
```

## Build the Simulation Package

```bash
# From your ROS2 workspace root (e.g., ~/ros2_ws)
cp -r ros-linux/ros2-sdk/fr5_pybullet_sim src/
colcon build --packages-select fr5_pybullet_sim
source install/setup.bash
```

## Launch Commands

| Purpose | Command |
|---------|---------|
| PyBullet sim (headless) | `ros2 launch fr5_pybullet_sim fr5_sim.launch.py gui:=false` |
| PyBullet sim (GUI) | `ros2 launch fr5_pybullet_sim fr5_sim.launch.py gui:=true` |
| PyBullet sim with gripper | `ros2 launch fr5_pybullet_sim fr5_sim.launch.py urdf_name:=fairino5_v6_with_ag95.urdf` |
| MoveIt2 full demo | `ros2 launch fairino5_v6_moveit2_config demo.launch.py` |

## Examples

### Basic

```bash
python ros-linux/examples/basic/01_first_example.py   # Check sim connectivity
python ros-linux/examples/basic/02_move_joint.py      # Move to home
python ros-linux/examples/basic/03_gripper_control.py # Open/close gripper
python ros-linux/examples/basic/04_read_state.py      # Subscribe /joint_states
```

### Advanced

```bash
python ros-linux/examples/advanced/rrt_pick_and_place.py
python ros-linux/examples/advanced/moveit2_demo.py
```

## Services (fr5_pybullet_sim node)

| Service | Type | Description |
|---------|------|-------------|
| `/fr5_sim/get_state` | Trigger | Check node is running |
| `/fr5_sim/set_joint_position` | Trigger | Reset to home |
| `/fr5_sim/open_gripper` | Trigger | Open AG-95 gripper |
| `/fr5_sim/close_gripper` | Trigger | Close AG-95 gripper |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Robot joint positions at ~120 Hz |
| `/tcp_pose` | geometry_msgs/PoseStamped | End-effector pose |
