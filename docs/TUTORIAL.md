# FR5 Robot Tutorial

Complete guide to using the FR5 robot with Python SDK, from basics to advanced features.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Basic Motion Control](#basic-motion-control)
3. [Gripper Control](#gripper-control)
4. [Workspace Management](#workspace-management)
5. [Cartesian Planning](#cartesian-planning)
6. [Trajectory Recording](#trajectory-recording)
7. [ROS2 Integration](#ros2-integration)
8. [Advanced Topics](#advanced-topics)

---

## Getting Started

### Installation

```bash
# Clone or navigate to FR-basics
cd /home/cflores18/FR-basics

# Install Python dependencies
pip install numpy pybullet

# For real robot support (optional)
cd python_sdk/real_robot/fairino
pip install .
```

### Your First Program

Create `my_first_robot.py`:

```python
#!/usr/bin/env python3
from fr5_robot_interface import FR5RobotInterface

# Connect to simulation
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    print("Robot connected!")

    # Move to home position
    robot.move_j([0, 0, 0, 0, 0, 0], velocity=30)

    # Get current state
    state = robot.get_state()
    print(f"Current position: {state['joint_positions']}")

print("Done!")
```

Run it:
```bash
python3 my_first_robot.py
```

---

## Basic Motion Control

### Joint Space Motion (MoveJ)

Move directly in joint space - fastest path between configurations.

```python
from fr5_robot_interface import FR5RobotInterface

with FR5RobotInterface(mode='simulation', gui=True) as robot:
    # Define target joint positions (in degrees)
    target = [30, -20, 45, -60, 0, 30]

    # Move to target
    result = robot.move_j(target, velocity=40)

    if result == 0:
        print("Motion successful!")
    else:
        print("Motion failed (collision detected)")
```

**Key Points:**
- Joint positions in **degrees**
- Velocity: 1-100 (percentage of max speed)
- Automatic collision detection with RRT-Connect
- Returns 0 on success, -1 on failure

### Understanding Joint Positions

The FR5 has 6 joints:

```
Position: [J1, J2, J3, J4, J5, J6]
Example:  [30, -20, 45, -60, 0, 30]

J1: Base rotation (±175°)
J2: Shoulder   (-265° to +85°)
J3: Elbow      (±162°)
J4: Wrist 1    (-265° to +85°)
J5: Wrist 2    (±175°)
J6: Wrist 3    (±175°)
```

### Getting Robot State

```python
state = robot.get_state()

print(f"Joint positions: {state['joint_positions']}")
print(f"Joint velocities: {state['joint_velocities']}")
print(f"TCP pose: {state['tcp_pose']}")
print(f"Gripper state: {state['gripper_state']}")
```

### Common Positions

```python
# Home position
HOME = [0, 0, 0, 0, 0, 0]

# Upright position
UPRIGHT = [0, -45, 90, -45, 0, 0]

# Folded position (compact)
FOLDED = [0, -90, 135, -45, 0, 0]
```

---

## Gripper Control

The FR5 uses a **DH AG-95** parallel gripper:
- **Stroke:** 95mm (0-95mm opening)
- **Force:** Up to 45N per finger
- **Resolution:** ~0.1mm

### Basic Open/Close

```python
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    # Open gripper fully
    robot.open_gripper()
    time.sleep(1)

    # Close gripper
    robot.close_gripper()
    time.sleep(1)
```

### Precise Position Control

```python
# Set gripper to specific opening (0-100%)
robot.move_gripper(50, speed_pct=30)  # 50% open (47.5mm)

# Different positions
robot.move_gripper(25, speed_pct=20)   # 25% - narrow grip
robot.move_gripper(75, speed_pct=40)   # 75% - wide grip
robot.move_gripper(100, speed_pct=50)  # 100% - fully open
```

### Gripper State Monitoring

```python
state = robot.get_state()
gripper = state['gripper_state']

if gripper:
    print(f"Position: {gripper['position_percent']:.1f}%")
    print(f"Opening: {gripper['opening_mm']:.1f}mm")
    print(f"Is open: {gripper['is_open']}")
    print(f"Is closed: {gripper['is_closed']}")
    print(f"Contact: {gripper['contact_detected']}")
```

### Pick and Place Example

```python
import time

with FR5RobotInterface(mode='simulation', gui=True) as robot:
    # 1. Open gripper and approach
    robot.open_gripper()
    robot.move_j([0, -30, 45, -60, 0, 0], velocity=40)

    # 2. Lower to object
    robot.move_j([0, -40, 60, -80, 0, 0], velocity=30)
    time.sleep(0.5)

    # 3. Grasp object (30mm gap for 60mm object)
    robot.move_gripper(30, speed_pct=20)
    time.sleep(1)

    # 4. Lift
    robot.move_j([0, -30, 45, -60, 0, 0], velocity=30)

    # 5. Move to place location
    robot.move_j([45, -30, 45, -60, 0, 45], velocity=40)

    # 6. Lower
    robot.move_j([45, -40, 60, -80, 0, 45], velocity=30)
    time.sleep(0.5)

    # 7. Release
    robot.open_gripper()
    time.sleep(1)

    # 8. Retract and return home
    robot.move_j([45, -30, 45, -60, 0, 45], velocity=30)
    robot.move_j([0, 0, 0, 0, 0, 0], velocity=40)
```

---

## Workspace Management

### Checking Workspace Limits

Always validate positions before moving:

```python
target = [45, -30, 60, -90, 0, 30]

# Check if position is valid
valid, message = robot.check_workspace_limits(target)

if valid:
    print(f"✓ Position is valid: {message}")
    robot.move_j(target, velocity=40)
else:
    print(f"✗ Position invalid: {message}")
```

### FR5 Joint Limits

```python
# Joint limits (degrees)
JOINT_LIMITS = {
    'J1': (-175, 175),   # Base rotation
    'J2': (-265, 85),    # Shoulder
    'J3': (-162, 162),   # Elbow
    'J4': (-265, 85),    # Wrist 1
    'J5': (-175, 175),   # Wrist 2
    'J6': (-175, 175),   # Wrist 3
}
```

### Cartesian Workspace

```python
# Cartesian limits
CARTESIAN_LIMITS = {
    'x_range': (-0.9, 0.9),      # meters
    'y_range': (-0.9, 0.9),      # meters
    'z_range': (0.0, 1.2),       # meters
    'radius_min': 0.15,          # minimum reach
    'radius_max': 0.85,          # maximum reach (0.922m spec - margin)
}
```

### Safe Motion Pattern

```python
def safe_move(robot, target, velocity=40):
    """Safely move to target with validation"""
    # Check workspace limits
    valid, msg = robot.check_workspace_limits(target)
    if not valid:
        print(f"❌ Invalid position: {msg}")
        return False

    # Execute motion
    result = robot.move_j(target, velocity)
    if result == 0:
        print("✓ Motion successful")
        return True
    else:
        print("❌ Motion failed (collision)")
        return False

# Usage
safe_move(robot, [30, -20, 45, -60, 0, 30])
```

---

## Cartesian Planning

Move the end-effector in straight lines through workspace.

### Basic Cartesian Interpolation

```python
from cartesian_planner import CartesianPlanner

# Define start and end poses [x, y, z, rx, ry, rz]
start = [0.5, 0.0, 0.4, 3.14, 0, 0]
end = [0.5, 0.2, 0.4, 3.14, 0, 0]

# Interpolate straight line
path = CartesianPlanner.interpolate_pose(start, end, num_points=30)

print(f"Generated {len(path)} waypoints")
```

### IK-Based Cartesian Motion

```python
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    # Get current state
    current_joints = robot.get_joint_positions()

    # Define target pose
    target_pose = [0.5, 0.15, 0.5, 3.14, 0, 0]

    # Plan Cartesian path
    if hasattr(robot.backend, 'robot_id'):
        joint_path = CartesianPlanner.plan_line(
            robot.backend.robot_id,
            robot.backend.ee_link_index,
            target_pose,
            current_joints,
            num_waypoints=20
        )

        if joint_path:
            # Validate path
            is_valid = CartesianPlanner.validate_cartesian_path(joint_path)

            if is_valid:
                # Execute path
                for joints in joint_path:
                    robot.move_j(joints.tolist(), velocity=30)
```

### Drawing Shapes

**Square:**
```python
import numpy as np

# Define square corners
corners = [
    [0.4, -0.1, 0.4, 3.14, 0, 0],
    [0.4,  0.1, 0.4, 3.14, 0, 0],
    [0.6,  0.1, 0.4, 3.14, 0, 0],
    [0.6, -0.1, 0.4, 3.14, 0, 0],
    [0.4, -0.1, 0.4, 3.14, 0, 0],
]

# Interpolate each edge
full_path = []
for i in range(len(corners) - 1):
    edge = CartesianPlanner.interpolate_pose(corners[i], corners[i+1], 15)
    full_path.extend(edge)
```

**Circle:**
```python
center = [0.5, 0.0, 0.4]
radius = 0.1
num_points = 40

circle_poses = []
for i in range(num_points):
    angle = 2 * np.pi * i / num_points
    x = center[0] + radius * np.cos(angle)
    y = center[1] + radius * np.sin(angle)
    z = center[2]
    pose = [x, y, z, 3.14, 0, 0]
    circle_poses.append(pose)
```

---

## Trajectory Recording

Record and replay robot motions - perfect for teaching by demonstration.

### Recording a Trajectory

```python
from cartesian_planner import TrajectoryRecorder
import time

recorder = TrajectoryRecorder()

with FR5RobotInterface(mode='simulation', gui=True) as robot:
    # Start recording
    recorder.start_recording('demo_motion')

    # Execute motions and record waypoints
    positions = [
        [0, 0, 0, 0, 0, 0],
        [30, -20, 30, -45, 0, 30],
        [-30, -20, 30, -45, 0, -30],
        [0, 0, 0, 0, 0, 0],
    ]

    for pos in positions:
        robot.move_j(pos, velocity=40)
        time.sleep(0.5)

        # Record current position
        current = robot.get_joint_positions()
        recorder.record_waypoint(current, timestamp=time.time())

    # Stop recording
    num_points = recorder.stop_recording('demo_motion')
    print(f"Recorded {num_points} waypoints")
```

### Saving and Loading

```python
# Save trajectory to disk
recorder.save_trajectory('demo_motion', 'my_trajectory.npy')

# Load trajectory later
new_recorder = TrajectoryRecorder()
name = new_recorder.load_trajectory('my_trajectory.npy')
print(f"Loaded: {name}")
```

### Playing Back Trajectories

```python
# Get trajectory
trajectory = recorder.get_trajectory('demo_motion')

# Replay it
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    for waypoint in trajectory:
        robot.move_j(waypoint.tolist(), velocity=50)
        time.sleep(0.2)
```

### Managing Multiple Trajectories

```python
# List all trajectories
all_trajs = recorder.list_trajectories()
for name in all_trajs:
    traj = recorder.get_trajectory(name)
    print(f"{name}: {len(traj)} waypoints")

# Delete a trajectory
recorder.clear_trajectory('old_motion')

# Clear all
recorder.clear_all()
```

---

## ROS2 Integration

Use FR5 robot through ROS2 services and topics.

### Starting the ROS2 Node

```bash
# Terminal 1: Start simulation node
ros2 run fr5_pybullet_sim sim_node

# Or use tmux for multi-pane view
cd /home/cflores18/fr5_ws/scripts
./launch_with_tmux.sh sim
```

### Available Services

```bash
# List services
ros2 service list

# Output:
# /set_joint_position
# /gripper_control
```

### Calling Services

**Move joints:**
```bash
ros2 service call /set_joint_position fr5_msgs/srv/SetJointPosition \
  "{joint_positions: [0.0, -0.523, 0.785, -1.047, 0.0, 0.0]}"
```

**Control gripper:**
```bash
# Open (100%)
ros2 service call /gripper_control fr5_msgs/srv/GripperControl \
  "{position: 100}"

# Close (0%)
ros2 service call /gripper_control fr5_msgs/srv/GripperControl \
  "{position: 0}"
```

### Monitoring Topics

**Joint states:**
```bash
ros2 topic echo /joint_states
```

**TCP pose:**
```bash
ros2 topic echo /tcp_pose
```

### Python Client

```python
import rclpy
from rclpy.node import Node
from fr5_msgs.srv import SetJointPosition, GripperControl

rclpy.init()
node = Node('fr5_client')

# Create clients
joint_client = node.create_client(SetJointPosition, '/set_joint_position')
gripper_client = node.create_client(GripperControl, '/gripper_control')

# Wait for services
joint_client.wait_for_service(timeout_sec=5.0)
gripper_client.wait_for_service(timeout_sec=5.0)

# Move robot
req = SetJointPosition.Request()
req.joint_positions = [0.0, -0.523, 0.785, -1.047, 0.0, 0.0]
future = joint_client.call_async(req)
rclpy.spin_until_future_complete(node, future)

if future.result().success:
    print("Motion completed!")

# Control gripper
req = GripperControl.Request()
req.position = 50
future = gripper_client.call_async(req)
rclpy.spin_until_future_complete(node, future)

node.destroy_node()
rclpy.shutdown()
```

---

## Advanced Topics

### Switching Between Simulation and Real Robot

```python
# Simulation
robot_sim = FR5RobotInterface(mode='simulation', gui=True)

# Real robot
robot_real = FR5RobotInterface(
    mode='real',
    robot_ip='192.168.58.2'
)

# Same code works for both!
robot.move_j([0, 0, 0, 0, 0, 0], velocity=30)
robot.open_gripper()
```

### Custom Trajectory Generation

```python
from trajectory_interpolator import TrajectoryInterpolator
import numpy as np

# Generate minimum-jerk trajectory
q_start = np.array([0, 0, 0, 0, 0, 0])
q_end = np.array([30, -20, 45, -60, 0, 30])

trajectory = TrajectoryInterpolator.minimum_jerk(q_start, q_end, num_points=100)

# Execute trajectory
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    for waypoint in trajectory:
        robot.backend._exec_to_q(waypoint, duration=0.05)
```

### Collision Detection

Automatic collision detection using RRT-Connect:

```python
# Collision checking is automatic in move_j()
result = robot.move_j(target, velocity=40)

if result == -1:
    print("Collision detected - motion aborted")
    # Try alternative path or adjust target
```

### Performance Tuning

```python
# Adjust simulation speed
robot.backend.dt = 1/240.0  # 240 Hz (faster)
robot.backend.dt = 1/60.0   # 60 Hz (slower, more stable)

# Tune PID gains for smoothness
pos_gain = 0.3   # Default: good balance
vel_gain = 0.8   # Default: good balance
force = 200      # Default: good for most cases

# Lower values = softer motion
# Higher values = more responsive (may oscillate)
```

---

## Common Patterns

### Safe Startup Sequence

```python
def startup_sequence(robot):
    """Safe robot startup"""
    print("Starting robot...")

    # Move to home
    robot.move_j([0, 0, 0, 0, 0, 0], velocity=20)

    # Open gripper
    if robot.gripper_controller:
        robot.open_gripper()

    # Move to ready position
    robot.move_j([0, -30, 45, -60, 0, 0], velocity=30)

    print("Robot ready!")
```

### Error Handling

```python
def robust_move(robot, target, max_retries=3):
    """Move with retry logic"""
    for attempt in range(max_retries):
        # Validate
        valid, msg = robot.check_workspace_limits(target)
        if not valid:
            print(f"Invalid: {msg}")
            return False

        # Execute
        result = robot.move_j(target, velocity=40)
        if result == 0:
            return True

        print(f"Attempt {attempt+1} failed, retrying...")
        time.sleep(0.5)

    return False
```

---

## Next Steps

1. **Run Examples:** Try all scripts in `/home/cflores18/FR-basics/examples/`
2. **Modify Examples:** Change positions, speeds, and parameters
3. **Run Tests:** Execute test suite: `pytest tests/`
4. **Build Applications:** Use SDK as foundation for your project
5. **Real Robot:** Deploy to physical FR5 robot

---

**Need Help?**
- Check `/home/cflores18/FR-basics/IMPLEMENTATION_SUMMARY.md`
- Review test files in `/home/cflores18/FR-basics/tests/`
- Examine example scripts in `/home/cflores18/FR-basics/examples/`
