# FR5 Robot ROS 2 Simulation

This folder contains the PyBullet simulation for the FR5 robot with RRT-Connect path planning, integrated with ROS 2.

## Features

- PyBullet physics simulation with realistic robot dynamics
- RRT-Connect collision-free path planning
- ROS 2 integration with joint state publishing
- Service-based motion control
- Environment with table, target block, and obstacle

## Quick Start

### Method 1: Standalone (No ROS 2 Required)

The simplest way to run the simulation:

```bash
cd /home/cflores18/FR-basics/ros2_sdk/simulation
python3 first_example.py
```

This will:
1. Start PyBullet GUI with the FR5 robot on a table
2. Add a target block (blue) and obstacle (red cylinder)
3. Execute pick-and-place motion using RRT-Connect planning

### Method 2: ROS 2 Node

For full ROS 2 integration:

1. **Build the workspace:**
```bash
cd /home/cflores18/FR-basics/ros2_sdk
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fr5_pybullet_sim
```

2. **Source the workspace:**
```bash
source install/setup.bash
```

3. **Run the simulation node:**
```bash
ros2 run fr5_pybullet_sim sim_node
```

4. **Control the robot via services (in another terminal):**
```bash
# Source ROS 2 first
source /opt/ros/jazzy/setup.bash
source /path/to/ros2_sdk/install/setup.bash

# Move to home position
ros2 service call /fr5_sim/go_home std_srvs/srv/Trigger

# Move above target block
ros2 service call /fr5_sim/go_to_target std_srvs/srv/Trigger

# Move to place position (other side of obstacle)
ros2 service call /fr5_sim/go_to_place std_srvs/srv/Trigger
```

Or run the example client:
```bash
python3 first_example.py --ros2
```

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions, velocities, efforts |
| `/tcp_pose` | `geometry_msgs/PoseStamped` | Tool center point pose |

## ROS 2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/fr5_sim/go_home` | `std_srvs/Trigger` | Return to home configuration |
| `/fr5_sim/go_to_target` | `std_srvs/Trigger` | Move above target block (blue) |
| `/fr5_sim/go_to_place` | `std_srvs/Trigger` | Move to place position |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gui` | `true` | Enable PyBullet GUI visualization |
| `urdf_name` | `fairino5_v6.urdf` | Robot URDF file name |
| `sim_rate` | `120.0` | Simulation timestep rate (Hz) |

## Files

- `first_example.py` - Main example script (standalone or ROS 2 client)
- `launch/fr5_sim.launch.py` - ROS 2 launch file
- `urdf/` - Robot URDF and mesh files

## Dependencies

- Python 3.10+
- pybullet
- numpy
- ROS 2 Jazzy (for ROS 2 mode)

Install Python dependencies:
```bash
pip install pybullet numpy
```

## Trajectory Planning

The simulation uses RRT-Connect (Rapidly-exploring Random Trees Connect) for collision-free path planning:

- Bidirectional tree growth from start and goal
- Collision checking against obstacles
- Path shortcutting for smoother trajectories
- Interpolation for continuous motion

## Algorithm Details

The simulation follows the trajectory planning approach from `Final_trajectory_planning.py`:

1. **Environment Setup**: Creates simulation with table, obstacles, and target
2. **RRT-Connect Planning**: Builds bidirectional trees from start and goal
3. **Path Shortcutting**: Removes unnecessary waypoints
4. **Path Interpolation**: Adds intermediate points for smooth execution
5. **Motion Execution**: Follows planned path with position control
