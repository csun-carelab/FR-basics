# FR5 Robot API Reference

Complete API documentation for the FR5 Python SDK.

## Table of Contents

1. [FR5RobotInterface](#fr5robotinterface)
2. [TrajectoryInterpolator](#trajectoryinterpolator)
3. [WorkspaceBoundary](#workspaceboundary)
4. [GripperController](#grippercontroller)
5. [CartesianPlanner](#cartesianplanner)
6. [TrajectoryRecorder](#trajectoryrecorder)
7. [ROS2 Services](#ros2-services)

---

## FR5RobotInterface

Unified interface for controlling FR5 robot in both simulation and real modes.

### Constructor

```python
FR5RobotInterface(mode='simulation', gui=True, robot_ip='192.168.58.2')
```

**Parameters:**
- `mode` (str): Operating mode - `'simulation'` or `'real'`
- `gui` (bool): Show PyBullet GUI (simulation mode only). Default: `True`
- `robot_ip` (str): Robot IP address (real mode only). Default: `'192.168.58.2'`

**Returns:**
- `FR5RobotInterface` object

**Example:**
```python
# Simulation with GUI
robot = FR5RobotInterface(mode='simulation', gui=True)

# Simulation headless
robot = FR5RobotInterface(mode='simulation', gui=False)

# Real robot
robot = FR5RobotInterface(mode='real', robot_ip='192.168.58.2')
```

### Context Manager

```python
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    robot.move_j([0, 0, 0, 0, 0, 0])
# Automatically disconnects
```

---

### Motion Methods

#### `move_j(joint_positions, velocity=40)`

Move robot in joint space (direct joint control).

**Parameters:**
- `joint_positions` (list[float]): Target joint angles in **degrees** [J1, J2, J3, J4, J5, J6]
- `velocity` (int): Motion speed percentage (1-100). Default: 40

**Returns:**
- `int`: 0 on success, -1 on failure (collision detected or invalid position)

**Features:**
- Automatic collision detection (RRT-Connect path planning)
- Minimum-jerk trajectory interpolation for smooth motion
- Workspace boundary validation

**Example:**
```python
# Move to position at 40% speed
result = robot.move_j([30, -20, 45, -60, 0, 30], velocity=40)

if result == 0:
    print("Success!")
else:
    print("Failed - collision or invalid position")
```

---

### Gripper Methods

#### `open_gripper(duration=1.0, force=None)`

Open gripper to maximum width (95mm for DH AG-95).

**Parameters:**
- `duration` (float): Time to complete motion in seconds. Default: 1.0
- `force` (float, optional): Maximum force per finger in Newtons (0-45N). Default: uses controller default

**Returns:**
- `int`: 0 on success, -1 on failure

**Example:**
```python
robot.open_gripper(duration=1.5, force=30)
```

---

#### `close_gripper(duration=1.0, force=None, detect_contact=True)`

Close gripper completely or until contact detected.

**Parameters:**
- `duration` (float): Time to complete motion in seconds. Default: 1.0
- `force` (float, optional): Maximum force per finger in Newtons (0-45N)
- `detect_contact` (bool): Stop closing if object contact detected. Default: True

**Returns:**
- `int`: 0 on success, -1 on failure

**Example:**
```python
# Close with contact detection
robot.close_gripper(duration=1.0, force=25, detect_contact=True)
```

---

#### `move_gripper(position_pct, speed_pct=30, force_pct=50)`

Move gripper to specific position (percentage of full stroke).

**Parameters:**
- `position_pct` (float): Target position 0-100% (0=closed, 100=fully open 95mm)
- `speed_pct` (float): Motion speed 1-100%. Default: 30
- `force_pct` (float): Gripping force 1-100%. Default: 50

**Returns:**
- `int`: 0 on success, -1 on failure

**Position Reference:**
- `0%` = Fully closed (0mm opening)
- `25%` = 23.75mm opening (narrow grip)
- `50%` = 47.5mm opening (medium grip)
- `75%` = 71.25mm opening (wide grip)
- `100%` = 95mm opening (fully open)

**Example:**
```python
# Set to 50% open (47.5mm)
robot.move_gripper(50, speed_pct=30, force_pct=60)

# Narrow grip for small objects
robot.move_gripper(25, speed_pct=20, force_pct=40)
```

---

### State Methods

#### `get_joint_positions()`

Get current joint angles.

**Returns:**
- `list[float]`: Current joint positions in degrees [J1, J2, J3, J4, J5, J6]

**Example:**
```python
joints = robot.get_joint_positions()
print(f"Current position: {joints}")
# Output: [0.0, -30.0, 45.0, -60.0, 0.0, 0.0]
```

---

#### `get_state()`

Get complete robot state.

**Returns:**
- `dict`: Dictionary containing:
  - `joint_positions` (list[float]): Joint angles in degrees
  - `joint_velocities` (list[float]): Joint velocities in deg/s
  - `tcp_pose` (list[float]): End-effector pose [x, y, z, rx, ry, rz]
  - `gripper_state` (dict or None): Gripper state information

**Gripper State Dictionary:**
```python
{
    'position_percent': 50.0,         # 0-100%
    'opening_mm': 47.5,               # 0-95mm
    'is_open': False,                 # True if > 70%
    'is_closed': False,               # True if < 10%
    'contact_detected': False,        # Object contact
    'forces': [10.5, 10.3]           # Forces per finger (N)
}
```

**Example:**
```python
state = robot.get_state()

print(f"Joints: {state['joint_positions']}")
print(f"TCP: {state['tcp_pose']}")

if state['gripper_state']:
    gs = state['gripper_state']
    print(f"Gripper: {gs['opening_mm']:.1f}mm")
    print(f"Contact: {gs['contact_detected']}")
```

---

### Validation Methods

#### `check_workspace_limits(joint_positions)`

Validate if joint configuration is within workspace limits.

**Parameters:**
- `joint_positions` (list[float]): Joint angles in degrees to validate

**Returns:**
- `tuple(bool, str)`: (is_valid, message)
  - `is_valid`: True if position is valid
  - `message`: Validation message or error description

**Example:**
```python
target = [180, 0, 0, 0, 0, 0]  # Invalid - J1 exceeds +175°
valid, msg = robot.check_workspace_limits(target)

if valid:
    print(f"✓ Valid: {msg}")
    robot.move_j(target, velocity=40)
else:
    print(f"✗ Invalid: {msg}")
    # Output: ✗ Invalid: Joint 1 exceeds limit: 180.0° (max: ±175°)
```

---

### Connection Methods

#### `disconnect()`

Disconnect from robot and clean up resources.

**Example:**
```python
robot = FR5RobotInterface(mode='simulation', gui=True)
robot.move_j([0, 0, 0, 0, 0, 0])
robot.disconnect()

# Or use context manager (automatic cleanup)
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    robot.move_j([0, 0, 0, 0, 0, 0])
```

---

## TrajectoryInterpolator

Generate smooth trajectories using minimum-jerk interpolation.

### Class Methods

#### `minimum_jerk(q_start, q_end, num_points=100)`

Generate minimum-jerk trajectory between two configurations.

**Parameters:**
- `q_start` (np.ndarray): Starting configuration (joint angles in degrees)
- `q_end` (np.ndarray): Ending configuration (joint angles in degrees)
- `num_points` (int): Number of interpolation points. Default: 100

**Returns:**
- `np.ndarray`: Array of shape (num_points, num_joints) containing trajectory

**Mathematical Formula:**
```
s(t) = 10t³ - 15t⁴ + 6t⁵  where t ∈ [0, 1]
```

**Properties:**
- Zero velocity at start and end
- Zero acceleration at start and end
- Natural, smooth motion profile
- Minimum jerk (derivative of acceleration)

**Example:**
```python
from trajectory_interpolator import TrajectoryInterpolator
import numpy as np

q_start = np.array([0, 0, 0, 0, 0, 0])
q_end = np.array([30, -20, 45, -60, 0, 30])

trajectory = TrajectoryInterpolator.minimum_jerk(q_start, q_end, num_points=100)

print(f"Shape: {trajectory.shape}")  # (100, 6)
print(f"Start: {trajectory[0]}")     # [0, 0, 0, 0, 0, 0]
print(f"End: {trajectory[-1]}")      # [30, -20, 45, -60, 0, 30]
```

---

## WorkspaceBoundary

Workspace validation and boundary checking.

### Constants

```python
# Joint limits (degrees)
JOINT_LIMITS = [
    (-175, 175),   # J1
    (-265, 85),    # J2
    (-162, 162),   # J3
    (-265, 85),    # J4
    (-175, 175),   # J5
    (-175, 175),   # J6
]

# Cartesian limits
CARTESIAN_LIMITS = {
    'x': (-0.9, 0.9),           # meters
    'y': (-0.9, 0.9),           # meters
    'z': (0.0, 1.2),            # meters
    'radius_min': 0.15,         # minimum reach (m)
    'radius_max': 0.85,         # maximum reach (m)
}
```

### Class Methods

#### `validate_joint_position(joint_positions)`

Validate joint configuration against limits.

**Parameters:**
- `joint_positions` (list[float]): Joint angles in degrees

**Returns:**
- `tuple(bool, str)`: (is_valid, message)

**Example:**
```python
from workspace_boundaries import WorkspaceBoundary

valid, msg = WorkspaceBoundary.validate_joint_position([180, 0, 0, 0, 0, 0])
print(f"{valid}: {msg}")
# Output: False: Joint 1 exceeds limit: 180.0° (max: ±175°)
```

---

#### `validate_cartesian_position(position)`

Validate Cartesian position against workspace envelope.

**Parameters:**
- `position` (list[float]): [x, y, z] position in meters

**Returns:**
- `tuple(bool, str)`: (is_valid, message)

**Example:**
```python
valid, msg = WorkspaceBoundary.validate_cartesian_position([0.5, 0.2, 0.4])
print(f"{valid}: {msg}")
```

---

#### `clip_to_limits(joint_positions)`

Clamp joint angles to valid range.

**Parameters:**
- `joint_positions` (list[float]): Joint angles in degrees

**Returns:**
- `list[float]`: Clamped joint positions

**Example:**
```python
clamped = WorkspaceBoundary.clip_to_limits([180, 0, 0, 0, 0, 0])
print(clamped)
# Output: [175.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # J1 clamped to 175°
```

---

## GripperController

DH AG-95 parallel gripper controller.

### Constants

```python
GRIPPER_STROKE_MM = 95        # Total stroke in millimeters
GRIPPER_STROKE_M = 0.095      # Total stroke in meters
MAX_FORCE_PER_FINGER = 45     # Maximum force per finger (N)
```

### Constructor

```python
GripperController(robot_id, finger_joint_indices, dt=1/120.0)
```

**Parameters:**
- `robot_id` (int): PyBullet robot body ID
- `finger_joint_indices` (list[int]): List of finger joint indices
- `dt` (float): Simulation time step. Default: 1/120.0

---

### Methods

#### `open(duration=1.0, force=None)`

Open gripper to full width (95mm).

**Parameters:**
- `duration` (float): Time to complete in seconds
- `force` (float, optional): Max force per finger (0-45N)

**Returns:**
- `int`: 0 on success

---

#### `close(duration=1.0, force=None, detect_contact=True)`

Close gripper with optional contact detection.

**Parameters:**
- `duration` (float): Time to complete in seconds
- `force` (float, optional): Max force per finger (0-45N)
- `detect_contact` (bool): Stop if object detected

**Returns:**
- `int`: 0 on success

---

#### `set_position_mm(position_mm, duration=0.5, force=None)`

Set gripper to specific opening width.

**Parameters:**
- `position_mm` (float): Opening width in millimeters (0-95mm)
- `duration` (float): Time to complete
- `force` (float, optional): Max force per finger

**Returns:**
- `int`: 0 on success

---

#### `get_state()`

Get current gripper state.

**Returns:**
- `dict`: Gripper state with keys:
  - `position_percent`: Opening percentage (0-100%)
  - `opening_mm`: Opening in millimeters (0-95mm)
  - `is_open`: True if > 70% open
  - `is_closed`: True if < 10% open
  - `contact_detected`: True if object contact detected
  - `forces`: List of forces per finger (N)

---

## CartesianPlanner

Cartesian space path planning and interpolation.

### Class Methods

#### `interpolate_pose(pose_start, pose_end, num_points=50)`

Linearly interpolate between two Cartesian poses.

**Parameters:**
- `pose_start` (list[float]): Starting pose [x, y, z, rx, ry, rz]
- `pose_end` (list[float]): Ending pose [x, y, z, rx, ry, rz]
- `num_points` (int): Number of interpolation points. Default: 50

**Returns:**
- `list[list[float]]`: List of interpolated poses

**Pose Format:**
- `[x, y, z, rx, ry, rz]`
- Position in meters, orientation in radians (Euler angles)

**Example:**
```python
from cartesian_planner import CartesianPlanner

start = [0.5, 0.0, 0.4, 3.14, 0, 0]
end = [0.5, 0.2, 0.4, 3.14, 0, 0]

path = CartesianPlanner.interpolate_pose(start, end, num_points=30)
print(f"Generated {len(path)} waypoints")
```

---

#### `plan_line(robot_id, ee_link_index, target_pose, current_joints, num_waypoints=30)`

Plan straight-line Cartesian path using inverse kinematics.

**Parameters:**
- `robot_id` (int): PyBullet robot body ID
- `ee_link_index` (int): End-effector link index
- `target_pose` (list[float]): Target pose [x, y, z, rx, ry, rz]
- `current_joints` (list[float]): Current joint configuration
- `num_waypoints` (int): Number of waypoints. Default: 30

**Returns:**
- `list[np.ndarray]` or `None`: List of joint configurations, or None if IK fails

**Example:**
```python
joint_path = CartesianPlanner.plan_line(
    robot_id=1,
    ee_link_index=6,
    target_pose=[0.5, 0.15, 0.5, 3.14, 0, 0],
    current_joints=[0, -30, 45, -60, 0, 0],
    num_waypoints=20
)

if joint_path:
    print(f"Path planned: {len(joint_path)} waypoints")
    for joints in joint_path:
        # Execute motion
        robot.move_j(joints.tolist(), velocity=30)
```

---

#### `validate_cartesian_path(path, max_joint_diff=0.5)`

Validate Cartesian path for smoothness.

**Parameters:**
- `path` (list[np.ndarray]): List of joint configurations
- `max_joint_diff` (float): Max allowed joint change between waypoints (radians). Default: 0.5

**Returns:**
- `bool`: True if path is smooth and feasible

**Example:**
```python
is_valid = CartesianPlanner.validate_cartesian_path(joint_path, max_joint_diff=0.3)

if is_valid:
    print("Path is smooth - safe to execute")
else:
    print("Path has discontinuities - may cause jerky motion")
```

---

## TrajectoryRecorder

Record, save, and playback robot trajectories.

### Constructor

```python
TrajectoryRecorder()
```

**Example:**
```python
from cartesian_planner import TrajectoryRecorder

recorder = TrajectoryRecorder()
```

---

### Methods

#### `start_recording(name)`

Start recording a new trajectory.

**Parameters:**
- `name` (str): Name for this trajectory

**Example:**
```python
recorder.start_recording('pick_and_place_1')
```

---

#### `record_waypoint(joint_positions, timestamp=None)`

Record a waypoint during trajectory recording.

**Parameters:**
- `joint_positions` (list[float]): Current joint positions
- `timestamp` (float, optional): Optional timestamp. Default: sequential counter

**Example:**
```python
# During motion
for pos in positions:
    robot.move_j(pos, velocity=40)
    current = robot.get_joint_positions()
    recorder.record_waypoint(current, timestamp=time.time())
```

---

#### `stop_recording(name)`

Stop recording and save trajectory.

**Parameters:**
- `name` (str): Name to save trajectory under

**Returns:**
- `int`: Number of waypoints recorded

**Example:**
```python
num_waypoints = recorder.stop_recording('pick_and_place_1')
print(f"Recorded {num_waypoints} waypoints")
```

---

#### `get_trajectory(name)`

Get recorded trajectory by name.

**Parameters:**
- `name` (str): Trajectory name

**Returns:**
- `list[np.ndarray]` or `None`: List of joint configurations, or None if not found

**Example:**
```python
trajectory = recorder.get_trajectory('pick_and_place_1')

if trajectory:
    for waypoint in trajectory:
        robot.move_j(waypoint.tolist(), velocity=50)
```

---

#### `save_trajectory(name, filename)`

Save trajectory to disk.

**Parameters:**
- `name` (str): Trajectory name
- `filename` (str): Output filename (.npy format)

**Example:**
```python
recorder.save_trajectory('pick_and_place_1', 'my_trajectory.npy')
```

---

#### `load_trajectory(filename)`

Load trajectory from disk.

**Parameters:**
- `filename` (str): Input filename (.npy format)

**Returns:**
- `str`: Name of loaded trajectory

**Example:**
```python
name = recorder.load_trajectory('my_trajectory.npy')
print(f"Loaded trajectory: {name}")
```

---

#### `list_trajectories()`

List all recorded trajectory names.

**Returns:**
- `list[str]`: List of trajectory names

**Example:**
```python
all_trajectories = recorder.list_trajectories()
for name in all_trajectories:
    traj = recorder.get_trajectory(name)
    print(f"{name}: {len(traj)} waypoints")
```

---

#### `clear_trajectory(name)`

Delete a specific trajectory.

**Parameters:**
- `name` (str): Trajectory name to delete

---

#### `clear_all()`

Delete all recorded trajectories.

---

## ROS2 Services

FR5 robot ROS2 service interface.

### SetJointPosition Service

**Service Name:** `/set_joint_position`

**Service Type:** `fr5_msgs/srv/SetJointPosition`

**Request:**
```python
joint_positions: float64[6]  # Joint angles in radians
```

**Response:**
```python
success: bool                # True if motion succeeded
message: string              # Status message
```

**Command Line:**
```bash
ros2 service call /set_joint_position fr5_msgs/srv/SetJointPosition \
  "{joint_positions: [0.0, -0.523, 0.785, -1.047, 0.0, 0.0]}"
```

**Python Client:**
```python
from fr5_msgs.srv import SetJointPosition

client = node.create_client(SetJointPosition, '/set_joint_position')
client.wait_for_service()

req = SetJointPosition.Request()
req.joint_positions = [0.0, -0.523, 0.785, -1.047, 0.0, 0.0]

future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)

if future.result().success:
    print("Motion completed!")
```

---

### GripperControl Service

**Service Name:** `/gripper_control`

**Service Type:** `fr5_msgs/srv/GripperControl`

**Request:**
```python
position: int32              # Gripper position 0-100%
```

**Response:**
```python
success: bool                # True if command succeeded
message: string              # Status message
```

**Command Line:**
```bash
# Open (100%)
ros2 service call /gripper_control fr5_msgs/srv/GripperControl "{position: 100}"

# Close (0%)
ros2 service call /gripper_control fr5_msgs/srv/GripperControl "{position: 0}"

# Half open (50%)
ros2 service call /gripper_control fr5_msgs/srv/GripperControl "{position: 50}"
```

**Python Client:**
```python
from fr5_msgs.srv import GripperControl

client = node.create_client(GripperControl, '/gripper_control')
client.wait_for_service()

req = GripperControl.Request()
req.position = 50  # 50% open

future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)

if future.result().success:
    print("Gripper moved!")
```

---

### Topics

#### `/joint_states`

**Type:** `sensor_msgs/msg/JointState`

**Publishing Rate:** 50 Hz (every 0.02s)

**Content:**
```python
header:
  stamp: <timestamp>
name: ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # radians
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # rad/s
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # N⋅m (torque)
```

**Monitoring:**
```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

---

#### `/tcp_pose`

**Type:** `geometry_msgs/msg/PoseStamped`

**Publishing Rate:** 50 Hz (every 0.02s)

**Content:**
```python
header:
  stamp: <timestamp>
pose:
  position:
    x: 0.5      # meters
    y: 0.0
    z: 0.4
  orientation:
    x: 0.0      # quaternion
    y: 0.0
    z: 0.0
    w: 1.0
```

**Monitoring:**
```bash
ros2 topic echo /tcp_pose
ros2 topic hz /tcp_pose
```

---

## Type Conversions

### Degrees ↔ Radians

```python
import math

# Degrees to radians
radians = degrees * (math.pi / 180)

# Radians to degrees
degrees = radians * (180 / math.pi)

# Common conversions
30° = 0.523 rad
45° = 0.785 rad
60° = 1.047 rad
90° = 1.571 rad
```

### Position Percentage ↔ Millimeters (Gripper)

```python
# Percentage to millimeters (DH AG-95: 95mm stroke)
mm = (percent / 100.0) * 95.0

# Millimeters to percentage
percent = (mm / 95.0) * 100.0

# Examples:
0% = 0mm (fully closed)
25% = 23.75mm
50% = 47.5mm
75% = 71.25mm
100% = 95mm (fully open)
```

---

## Error Codes

### Motion Return Values

- `0`: Success - motion completed
- `-1`: Failure - collision detected or position invalid

### Common Error Messages

```
"Joint X exceeds limit: Y° (max: ±Z°)"
→ Joint position outside valid range

"Cartesian position outside workspace envelope"
→ Target position unreachable

"IK failed at waypoint"
→ Inverse kinematics couldn't solve for target pose

"Large joint jump detected"
→ Path has discontinuity (possible singularity)

"Position must be 0-100, got X"
→ Gripper position percentage out of range
```

---

## Best Practices

1. **Always validate positions:**
   ```python
   valid, msg = robot.check_workspace_limits(target)
   if valid:
       robot.move_j(target, velocity=40)
   ```

2. **Use context managers:**
   ```python
   with FR5RobotInterface(mode='simulation', gui=True) as robot:
       robot.move_j([0, 0, 0, 0, 0, 0])
   # Auto cleanup
   ```

3. **Check return values:**
   ```python
   result = robot.move_j(target, velocity=40)
   if result != 0:
       print("Motion failed!")
   ```

4. **Gradual velocity increases:**
   ```python
   # Start slow
   robot.move_j(pos1, velocity=20)
   # Increase speed
   robot.move_j(pos2, velocity=40)
   ```

5. **Safe gripper control:**
   ```python
   # Control force for delicate objects
   robot.move_gripper(30, speed_pct=20, force_pct=30)
   ```

---

**For more information, see:**
- Tutorial: `/home/cflores18/FR-basics/docs/TUTORIAL.md`
- Examples: `/home/cflores18/FR-basics/examples/`
- Tests: `/home/cflores18/FR-basics/tests/`
