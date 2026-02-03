# FR5 Robot Examples

This directory contains comprehensive examples demonstrating all features of the FR5 robot Python SDK and simulation environment.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Example Scripts](#example-scripts)
3. [Running Examples](#running-examples)
4. [Prerequisites](#prerequisites)
5. [Troubleshooting](#troubleshooting)

---

## Quick Start

```bash
# Run any example script
cd /home/cflores18/FR-basics/examples
python3 example_smooth_motion.py
```

---

## Example Scripts

### 1. `example_smooth_motion.py`

**Demonstrates:** Smooth motion with minimum-jerk trajectories

**Features:**
- Natural acceleration profiles
- Zero velocity at start/end points
- Tuned PID gains (pos: 0.3, vel: 0.8)
- Collision detection with RRT-Connect
- Multi-waypoint motion sequences

**What you'll see:**
- Robot moves through 5 waypoints
- Smooth, natural-looking motion
- No jerking or sudden accelerations
- Real-time position feedback

**Run:**
```bash
python3 example_smooth_motion.py
```

---

### 2. `example_workspace_boundaries.py`

**Demonstrates:** Workspace boundary validation and safety checking

**Features:**
- Joint limit validation (FR5 specific ranges)
- Cartesian workspace envelope checking
- Real-time boundary visualization
- Position clamping and validation

**FR5 Joint Limits:**
| Joint | Min (°) | Max (°) |
|-------|---------|---------|
| J1    | -175    | +175    |
| J2    | -265    | +85     |
| J3    | -162    | +162    |
| J4    | -265    | +85     |
| J5    | -175    | +175    |
| J6    | -175    | +175    |

**Cartesian Limits:**
- Maximum reach: 922mm
- Minimum reach: 150mm (safety buffer)
- Working envelope: 0.15m - 0.85m radius
- Height range: 0.0m - 1.2m

**Run:**
```bash
python3 example_workspace_boundaries.py
```

---

### 3. `example_gripper_control.py`

**Demonstrates:** DH AG-95 parallel gripper control

**Features:**
- Open/close operations (0-95mm stroke)
- Precise position control (0.1mm resolution)
- Variable speed control (1-100%)
- Force control (up to 45N per finger)
- Contact detection
- Complete pick-and-place cycle

**DH AG-95 Specifications:**
- **Stroke:** 95mm (0-95mm opening)
- **Force:** Up to 45N per finger (90N total)
- **Resolution:** ~0.1mm
- **Payload:** Up to 5kg
- **Type:** Parallel jaw gripper

**What you'll see:**
- Gripper opening/closing smoothly
- Precise position control at 25%, 50%, 75%, 100%
- Complete simulated pick-and-place operation
- Real-time state monitoring

**Run:**
```bash
python3 example_gripper_control.py
```

---

### 4. `example_cartesian_planning.py`

**Demonstrates:** Cartesian space motion planning (MoveL)

**Features:**
- Linear pose interpolation
- IK-based path generation
- Path smoothness validation
- Straight-line motion in workspace
- Complex patterns (squares, circles)

**Examples shown:**
1. Simple linear motion (20cm translation)
2. IK-based Cartesian path planning
3. Drawing a square path
4. Circular motion in XY plane

**Usage in code:**
```python
from cartesian_planner import CartesianPlanner

# Interpolate between poses
path = CartesianPlanner.interpolate_pose(start, end, num_points=30)

# Plan with IK
joint_path = CartesianPlanner.plan_line(
    robot_id, ee_link, target_pose, current_joints
)

# Validate path
is_valid = CartesianPlanner.validate_cartesian_path(path)
```

**Run:**
```bash
python3 example_cartesian_planning.py
```

---

### 5. `example_trajectory_recording.py`

**Demonstrates:** Trajectory recording, saving, and playback

**Features:**
- Record waypoints during motion
- Save/load trajectories (.npy format)
- List and manage multiple trajectories
- Replay recorded motions
- Teaching by demonstration
- Trajectory analysis (path length, statistics)

**What you'll see:**
1. Recording a pick-and-place trajectory (8 waypoints)
2. Saving trajectory to disk
3. Loading trajectory from file
4. Replaying recorded motion
5. Recording a scanning motion
6. Comparing multiple trajectories

**Usage in code:**
```python
from cartesian_planner import TrajectoryRecorder

recorder = TrajectoryRecorder()

# Record
recorder.start_recording('my_trajectory')
recorder.record_waypoint(joints)
recorder.stop_recording('my_trajectory')

# Save/Load
recorder.save_trajectory('my_trajectory', 'file.npy')
recorder.load_trajectory('file.npy')

# Playback
trajectory = recorder.get_trajectory('my_trajectory')
```

**Run:**
```bash
python3 example_trajectory_recording.py
```

---

### 6. `example_ros2_services.py`

**Demonstrates:** ROS2 service integration

**Note:** This is a documentation example showing ROS2 commands. It prints instructions rather than executing them.

**Features:**
- SetJointPosition service usage
- GripperControl service usage
- Topic monitoring (/joint_states, /tcp_pose)
- Service discovery
- Python client examples

**Prerequisites:**
Start the FR5 simulation ROS2 node first:
```bash
# Option 1: Direct launch
ros2 run fr5_pybullet_sim sim_node

# Option 2: tmux multi-pane (recommended)
cd /home/cflores18/fr5_ws/scripts
./launch_with_tmux.sh sim
```

**Service Examples:**

```bash
# Move to position
ros2 service call /set_joint_position fr5_msgs/srv/SetJointPosition \
  "{joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Control gripper
ros2 service call /gripper_control fr5_msgs/srv/GripperControl \
  "{position: 50}"

# Monitor joint states
ros2 topic echo /joint_states

# Monitor TCP pose
ros2 topic echo /tcp_pose
```

**Run:**
```bash
python3 example_ros2_services.py
```

---

## Running Examples

### Option 1: Run Individual Examples

```bash
cd /home/cflores18/FR-basics/examples

# Run any example
python3 example_smooth_motion.py
python3 example_workspace_boundaries.py
python3 example_gripper_control.py
python3 example_cartesian_planning.py
python3 example_trajectory_recording.py
python3 example_ros2_services.py
```

### Option 2: Make Executable and Run

```bash
chmod +x example_*.py
./example_smooth_motion.py
```

---

## Prerequisites

### System Requirements

- **OS:** Ubuntu 22.04+ or similar Linux distribution
- **Python:** 3.8+
- **ROS2:** Humble or Jazzy (for ROS2 examples only)

### Python Dependencies

```bash
# Install required packages
pip install numpy pybullet

# For real robot (optional)
cd /home/cflores18/FR-basics/python_sdk/real_robot/fairino
pip install .
```

### ROS2 Setup (Optional)

Only needed for `example_ros2_services.py`:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash  # or jazzy

# Build workspace
cd /home/cflores18/fr5_ws
colcon build

# Source workspace
source install/setup.bash
```

---

## Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'fr5_robot_interface'"

**Solution:**
The examples add the correct path automatically. Make sure you're running from the examples directory:
```bash
cd /home/cflores18/FR-basics/examples
python3 example_smooth_motion.py
```

### Issue: "pybullet.error: Cannot load URDF file"

**Solution:**
Check that the URDF path is correct. The code automatically patches URDF mesh paths. If issues persist:
```bash
# Verify URDF exists
ls /home/cflores18/ros2_ws/install/share/fr5_description/urdf/fr5_robot.urdf
```

### Issue: PyBullet GUI doesn't appear

**Solution:**
Make sure you have a display available. If running headless:
```python
# Modify the robot initialization
robot = FR5RobotInterface(mode='simulation', gui=False)
```

### Issue: Gripper not detected in simulation

**Solution:**
Ensure the URDF includes gripper joints. Check:
```bash
grep -i "finger\|gripper" /home/cflores18/ros2_ws/install/share/fr5_description/urdf/fr5_robot.urdf
```

### Issue: ROS2 services not available

**Solution:**
1. Make sure ROS2 node is running:
   ```bash
   ros2 run fr5_pybullet_sim sim_node
   ```

2. Check available services:
   ```bash
   ros2 service list
   ```

3. Verify workspace is sourced:
   ```bash
   source /home/cflores18/fr5_ws/install/setup.bash
   ```

---

## Performance Tips

### Simulation Speed

- For faster simulation, disable GUI: `gui=False`
- Reduce trajectory points for quicker planning
- Use lower velocity values for smoother motion

### Trajectory Quality

- Increase `num_waypoints` for smoother Cartesian paths
- Tune `max_joint_diff` in path validation for stricter smoothness
- Use higher `num_points` in minimum-jerk interpolation

### Gripper Control

- Adjust `speed_pct` for slower/faster gripper motion
- Use `force` parameter to control gripping strength
- Enable `detect_contact` for object detection

---

## Next Steps

After running these examples, you can:

1. **Modify Examples:** Experiment with different positions, speeds, and parameters
2. **Create Custom Scripts:** Use these as templates for your own applications
3. **Real Robot:** Switch to real robot mode by changing `mode='simulation'` to `mode='real'`
4. **ROS2 Integration:** Build more complex ROS2 applications using the services
5. **Advanced Features:** Explore collision detection, trajectory optimization, and path planning

---

## Additional Resources

- **Main Documentation:** `/home/cflores18/FR-basics/IMPLEMENTATION_SUMMARY.md`
- **Test Suite:** `/home/cflores18/FR-basics/tests/`
- **Python SDK:** `/home/cflores18/FR-basics/python_sdk/`
- **ROS2 Workspace:** `/home/cflores18/fr5_ws/`

---

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review test files in `/home/cflores18/FR-basics/tests/`
3. Examine the implementation summary

---

**Happy Coding!** 🤖
