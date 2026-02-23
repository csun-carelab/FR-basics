# FR5 ROS2 System Overview

**Robot:** Fairino FR5 — 6-DOF Collaborative Robot Arm
**ROS2 Distro:** Humble Hawksbill

---

## Quick Start

### 1. Build the ROS2 Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Run PyBullet Simulation

```bash
ros2 launch fr5_pybullet_sim fr5_sim.launch.py gui:=true
```

### 3. Run MoveIt2 Demo

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

---

## Packages

| Package | Type | Description |
|---------|------|-------------|
| `fairino_msgs` | ament_cmake | Custom messages and services for Fairino robots |
| `fairino_description` | ament_cmake | URDF model + STL meshes for FR5 |
| `fr5_pybullet_sim` | ament_python | PyBullet physics simulation with RRT-Connect planning |
| `fairino5_v6_moveit2_config` | ament_cmake | MoveIt2 motion planning configuration |
| `fairino_hardware` | ament_cmake | ros2_control hardware interface (requires proprietary `libfairino.so`) |

---

## Architecture

```
                         ┌──────────────────────────┐
                         │      ROS2 DDS Network     │
                         └──────────┬───────────────┘
                                    │
           ┌────────────────────────┼────────────────────────┐
           │                        │                        │
    ┌──────▼──────┐         ┌──────▼───────┐         ┌──────▼──────┐
    │  PyBullet    │         │   MoveIt2     │         │  Hardware    │
    │  Simulation  │         │   Stack       │         │  Driver      │
    │  (sim_node)  │         │  (demo.launch)│         │ (cmd_server) │
    └──────┬──────┘         └──────┬───────┘         └──────┬──────┘
           │                        │                        │
      PyBullet Engine        Mock Controllers          TCP to Robot
      (120 Hz physics)      (ros2_control)            Controller
```

---

## Launch Mode A: PyBullet Simulation

**Command:**
```bash
ros2 launch fr5_pybullet_sim fr5_sim.launch.py gui:=true
```

### Node: `/fr5_pybullet_sim`

A self-contained simulation node with physics, inverse kinematics, and collision-free path planning.

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `gui` | bool | `true` | Enable PyBullet GUI |
| `urdf_name` | string | `fairino5_v6.urdf` | URDF file from `fairino_description` |
| `sim_rate` | float | `120.0` | Physics simulation rate (Hz) |

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | ~50 Hz | Joint positions (rad), velocities (rad/s), efforts (Nm) for j1-j6 |
| `/tcp_pose` | `geometry_msgs/msg/PoseStamped` | ~50 Hz | End-effector pose in `base_link` frame (position xyz + quaternion) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/fr5_sim/go_home` | `std_srvs/srv/Trigger` | Move to home position `[0, -0.5, 0.5, -1.5, 0, 1.0]` rad |
| `/fr5_sim/go_to_target` | `std_srvs/srv/Trigger` | Plan and execute motion to target object (blue box) |
| `/fr5_sim/go_to_place` | `std_srvs/srv/Trigger` | Plan and execute motion to place position (avoids obstacle) |

**Calling services:**
```bash
ros2 service call /fr5_sim/go_home std_srvs/srv/Trigger
ros2 service call /fr5_sim/go_to_target std_srvs/srv/Trigger
ros2 service call /fr5_sim/go_to_place std_srvs/srv/Trigger
```

### Sample `/joint_states` Message

```yaml
header:
  stamp: {sec: ..., nanosec: ...}
  frame_id: ''
name: [j1, j2, j3, j4, j5, j6]
position: [0.0005, -0.4901, 0.4806, -1.5017, -0.0001, 0.9939]  # radians
velocity: [-2.76e-05, -7.48e-04, -1.48e-03, 3.97e-04, -3.10e-04, -4.48e-04]
effort: [0.197, -80.645, -27.971, -2.105, 0.451, 0.0004]  # Nm (gravity-loaded)
```

### Sample `/tcp_pose` Message

```yaml
header:
  stamp: {sec: ..., nanosec: ...}
  frame_id: base_link
pose:
  position: {x: 0.871, y: 0.179, z: 0.978}  # meters
  orientation: {x: -0.182, y: 0.684, z: 0.683, w: 0.181}  # quaternion
```

### Path Planning Features

- **Algorithm:** RRT-Connect (bidirectional, up to 5000 iterations)
- **Smoothing:** Shortcut optimization + cubic spline interpolation
- **Trajectory:** Minimum-jerk 5th-order polynomial velocity profiles
- **Collision:** PyBullet collision detection against table and obstacles

### Simulation Environment

| Object | Position | Description |
|--------|----------|-------------|
| Ground | z=0 | PyBullet ground plane |
| Table | [0.5, 0, 0] | Work surface |
| Target (blue) | [0.5, -0.25, tabletop+0.025] | Pick target |
| Obstacle (red) | [0.5, 0, tabletop+0.15] | Cylinder obstacle |

---

## Launch Mode B: MoveIt2 Motion Planning

**Command:**
```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

This launches a full MoveIt2 stack with mock (simulated) hardware.

### Nodes Started

| Node | Description |
|------|-------------|
| `/move_group` | MoveIt2 core — motion planning, IK, FK |
| `/robot_state_publisher` | Publishes TF transforms from URDF + joint states |
| `/controller_manager` | ros2_control controller lifecycle manager |
| `/fairino5_controller` | JointTrajectoryController for 6-DOF arm |
| `/joint_state_broadcaster` | Publishes joint states from hardware interface |
| `/moveit_simple_controller_manager` | MoveIt-to-ros2_control bridge |

### Published Topics

| Topic | Message Type | Publisher | Description |
|-------|-------------|-----------|-------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster | Joint positions from mock hardware |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | joint_state_broadcaster | Extended joint state info |
| `/tf` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | Dynamic transform tree |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher | Static transforms (base) |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | URDF XML as string |
| `/robot_description_semantic` | `std_msgs/msg/String` | move_group | SRDF XML as string |
| `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | move_group | Planned trajectory for visualization |
| `/display_contacts` | `visualization_msgs/msg/MarkerArray` | move_group | Collision contact points |
| `/motion_plan_request` | `moveit_msgs/msg/MotionPlanRequest` | move_group | Active planning request |
| `/monitored_planning_scene` | `moveit_msgs/msg/PlanningScene` | move_group | Current planning scene state |
| `/planning_scene` | `moveit_msgs/msg/PlanningScene` | (input) | Scene updates |
| `/collision_object` | `moveit_msgs/msg/CollisionObject` | (input) | Add/remove collision objects |
| `/attached_collision_object` | `moveit_msgs/msg/AttachedCollisionObject` | (input) | Attach objects to robot |
| `/fairino5_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | fairino5_controller | Controller tracking state |
| `/fairino5_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | (input) | Trajectory commands to controller |
| `/trajectory_execution_event` | `std_msgs/msg/String` | (events) | Execution status events |

### Subscribed Topics

| Topic | Subscriber Node | Description |
|-------|----------------|-------------|
| `/joint_states` | robot_state_publisher | Reads joint positions to compute TF |
| `/trajectory_execution_event` | move_group | Monitors execution status |
| `/fairino5_controller/joint_trajectory` | fairino5_controller | Receives trajectory commands |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/plan_kinematic_path` | `moveit_msgs/srv/GetMotionPlan` | Plan a collision-free path |
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Inverse kinematics solver |
| `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | Forward kinematics solver |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | Plan Cartesian straight-line path |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | Check if joint state is valid |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Modify the planning scene |
| `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Get current planning scene |
| `/clear_octomap` | `std_srvs/srv/Empty` | Clear 3D occupancy map |
| `/query_planner_interface` | `moveit_msgs/srv/QueryPlannerInterfaces` | List available planners |
| `/get_planner_params` | `moveit_msgs/srv/GetPlannerParams` | Get planner parameters |
| `/set_planner_params` | `moveit_msgs/srv/SetPlannerParams` | Set planner parameters |
| `/fairino5_controller/query_state` | `control_msgs/srv/QueryTrajectoryState` | Query controller state at time |

### Action Servers

| Action | Type | Node | Description |
|--------|------|------|-------------|
| `/move_action` | `moveit_msgs/action/MoveGroup` | move_group | Plan and execute motion |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | move_group | Execute a pre-planned trajectory |
| `/fairino5_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | fairino5_controller | Low-level trajectory execution |

### MoveIt2 Configuration

| Config File | Purpose |
|------------|---------|
| `kinematics.yaml` | IK solver: KDL plugin, 0.005 rad resolution, 5ms timeout |
| `joint_limits.yaml` | Velocity: 3.15-3.2 rad/s, Acceleration: 0.7 rad/s², scaling: 10% |
| `ros2_controllers.yaml` | 100 Hz update, position command interface |
| `moveit_controllers.yaml` | FollowJointTrajectory action interface |
| `pilz_cartesian_limits.yaml` | 1.0 m/s translation, 1.57 rad/s rotation |
| `initial_positions.yaml` | All joints start at 0 rad |

### Planning Group

- **Name:** `fairino5_v6_group`
- **Chain:** `base_link` -> `wrist3_link` (6 DOF)
- **Joints:** j1, j2, j3, j4, j5, j6
- **Predefined Poses:**
  - `pos1`: [2.109, -1.925, 1.890, -1.520, -1.434, 0] rad
  - `pos2`: [1.063, -1.689, 1.984, -1.824, -1.502, 0] rad

---

## Launch Mode C: Real Robot (Hardware Driver)

> **Note:** Requires the proprietary `libfairino.so` SDK library (not included in repo).

**Command:**
```bash
ros2 run fairino_hardware ros2_cmd_server
```

### Node: `fr_command_server`

Bridges ROS2 to the physical Fairino robot controller via TCP sockets.

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `nonrt_state_data` | `fairino_msgs/msg/RobotNonrtState` | Full robot state: joint positions, torques, Cartesian pose, I/O, errors, gripper state |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `fairino_remote_command_service` | `fairino_msgs/srv/RemoteCmdInterface` | Execute 500+ robot commands via string interface |
| `fairino_script_service` | `fairino_msgs/srv/RemoteScriptContent` | Send script content to robot controller |

### Command Examples

```bash
# Move joints (degrees, speed %)
ros2 service call fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'MoveJ([0, 0, 0, 0, 0, 0], 30)'}"

# Activate gripper
ros2 service call fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'ActGripper(1, 1)'}"

# Move gripper to position
ros2 service call fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'MoveGripper(1, 500, 50, 50, 30000, 0)'}"

# Get forward kinematics
ros2 service call fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'GetForwardKin([0, 0, 0, 0, 0, 0])'}"

# Get inverse kinematics
ros2 service call fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'GetInverseKin(0, [0,0,0,0,0,0], [-348.4,-76.8,330.5,178.8,-1.8,67.9])'}"
```

### Supported Command Categories (500+)

| Category | Example Commands |
|----------|-----------------|
| **Joint Motion** | MoveJ, ServoJ, SplineStart/PTP/End |
| **Cartesian Motion** | MoveL, MoveC, Circle, ServoCart, MoveCart |
| **Gripper** | ActGripper, MoveGripper, GetGripperMotionDone |
| **I/O Control** | SetDO, SetAO, GetDI, GetAI, WaitDI |
| **Kinematics** | GetForwardKin, GetInverseKin, GetInverseKinRef |
| **TCP/Tool** | SetToolCoord, GetTCPOffset, GetActualTCPPose |
| **Motion Control** | PauseMotion, ResumeMotion, StopMotion, StartJOG |
| **Safety** | SetAnticollision, SetCollisionStrategy, ResetAllError |
| **Programs** | ProgramLoad, ProgramRun, ProgramPause, ProgramStop |
| **Force/Torque** | FT_SetConfig, FT_Activate, FT_SetZero, FT_Guard |
| **Welding** | ARCStart, ARCEnd, WeldingSetCurrent, WeaveStart |
| **Drag Teach** | DragTeachSwitch, IsInDragTeach |

---

## Custom Message Types

### `fairino_msgs/msg/RobotNonrtState`

Full non-real-time robot state (103 fields). Key fields:

| Field Group | Fields | Type | Description |
|------------|--------|------|-------------|
| **Joint Positions** | j1_cur_pos..j6_cur_pos | float64 | Current joint angles (rad) |
| **Joint Torques** | j1_cur_tor..j6_cur_tor | float64 | Current joint torques (Nm) |
| **Cartesian Pose** | cart_x/y/z_cur_pos | float64 | TCP position (mm) |
| **Cartesian Orient.** | cart_a/b/c_cur_pos | float64 | TCP orientation (deg) |
| **Flange Pose** | flange_x/y/z/a/b/c_cur_pos | float64 | Flange frame pose |
| **Force/Torque** | ft_fx/fy/fz/tx/ty/tz_data | float64 | F/T sensor readings |
| **Digital I/O** | dgt_output/input_h/l | uint8 | Control box I/O states |
| **Robot Mode** | robot_mode | uint8 | Operating mode |
| **Motion Done** | robot_motion_done | uint8 | Motion completion flag |
| **Gripper Done** | grip_motion_done | uint8 | Gripper completion flag |
| **Error Codes** | main_error_code, sub_error_code | uint32 | Error information |
| **Emergency** | emg | uint8 | Emergency stop status |
| **Safety** | safetyboxsig[6] | uint8[] | Safety box signals |

### `fairino_msgs/srv/RemoteCmdInterface`

```
# Request
string cmd_str    # Command string, e.g. "MoveJ([0,0,0,0,0,0], 30)"
---
# Response
string cmd_res    # "0" = success, "-1" = failure
```

### `fairino_msgs/srv/RemoteScriptContent`

```
# Request
string line_str   # Script content line
---
# Response
string res        # "0" = success, "-1" = failure
```

---

## Robot Specifications (FR5)

| Property | Value |
|----------|-------|
| DOF | 6 revolute joints |
| Links | base_link, shoulder, upperarm, forearm, wrist1, wrist2, wrist3 |
| Joint Names | j1, j2, j3, j4, j5, j6 |
| Max Joint Velocity | 3.15-3.2 rad/s |
| Max Joint Acceleration | 0.7 rad/s² |
| Kinematic Chain | base_link → wrist3_link |
| IK Solver | KDL (MoveIt2) / PyBullet IK_DLS (sim) |
| Mesh Format | STL (7 link meshes) |

---

## Monitoring & Debugging

### Watch Joint States (any mode)
```bash
ros2 topic echo /joint_states
```

### Watch TCP Pose (sim only)
```bash
ros2 topic echo /tcp_pose
```

### Watch Robot State (real robot only)
```bash
ros2 topic echo nonrt_state_data
```

### Check Topic Publish Rate
```bash
ros2 topic hz /joint_states
```

### Inspect a Node
```bash
ros2 node info /fr5_pybullet_sim
ros2 node info /move_group
```

### List All Active Topics/Services
```bash
ros2 topic list -t
ros2 service list -t
ros2 action list -t
```

### TF Tree (MoveIt2 mode)
```bash
ros2 run tf2_tools view_frames
```

---

## Topic/Node Relationship Diagram

```
PyBullet Mode:
──────────────
  ┌─────────────────┐
  │ fr5_pybullet_sim │
  │                  │──▶ /joint_states [sensor_msgs/JointState]
  │  (PyBullet       │──▶ /tcp_pose [geometry_msgs/PoseStamped]
  │   physics @120Hz)│
  │                  │──● /fr5_sim/go_home [Trigger]
  │                  │──● /fr5_sim/go_to_target [Trigger]
  │                  │──● /fr5_sim/go_to_place [Trigger]
  └─────────────────┘

MoveIt2 Mode:
─────────────
  ┌──────────────────────┐        ┌─────────────────────┐
  │ joint_state_broadcaster│──▶    │ robot_state_publisher│
  │                       │ /joint │                      │──▶ /tf
  │ (reads mock hardware) │ _states│ (URDF + joints → TF) │──▶ /tf_static
  └──────────────────────┘   │    └──────────┬───────────┘──▶ /robot_description
                              │              ▲
                              └──────────────┘
                                    subscribes

  ┌───────────────────────┐
  │ fairino5_controller    │◀── /fairino5_controller/joint_trajectory
  │                        │──▶ /fairino5_controller/controller_state
  │ (JointTrajectory       │──● /fairino5_controller/follow_joint_trajectory [Action]
  │  Controller @100Hz)    │
  └───────────────────────┘

  ┌───────────────┐
  │  move_group    │──▶ /display_planned_path
  │                │──▶ /motion_plan_request
  │ (MoveIt2 core) │──● /plan_kinematic_path [Service]
  │                │──● /compute_ik [Service]
  │                │──● /compute_fk [Service]
  │                │──● /compute_cartesian_path [Service]
  │                │──● /move_action [Action]
  │                │──● /execute_trajectory [Action]
  └───────────────┘

Real Robot Mode:
────────────────
  ┌──────────────────┐
  │ fr_command_server │──▶ nonrt_state_data [RobotNonrtState]
  │                   │
  │ (TCP socket to    │──● fairino_remote_command_service [RemoteCmdInterface]
  │  robot controller)│──● fairino_script_service [RemoteScriptContent]
  └──────────────────┘

  Legend: ──▶ publishes topic   ──● provides service/action
```

---

## Available Launch Files

| Launch File | Package | Purpose | Command |
|------------|---------|---------|---------|
| `fr5_sim.launch.py` | fr5_pybullet_sim | PyBullet simulation | `ros2 launch fr5_pybullet_sim fr5_sim.launch.py gui:=true` |
| `demo.launch.py` | fairino5_v6_moveit2_config | Full MoveIt2 demo | `ros2 launch fairino5_v6_moveit2_config demo.launch.py` |
| `move_group.launch.py` | fairino5_v6_moveit2_config | MoveIt2 planning only | `ros2 launch fairino5_v6_moveit2_config move_group.launch.py` |
| `rsp.launch.py` | fairino5_v6_moveit2_config | Robot state publisher | `ros2 launch fairino5_v6_moveit2_config rsp.launch.py` |
| `spawn_controllers.launch.py` | fairino5_v6_moveit2_config | Spawn ros2_control controllers | `ros2 launch fairino5_v6_moveit2_config spawn_controllers.launch.py` |
| `static_virtual_joint_tfs.launch.py` | fairino5_v6_moveit2_config | Static TF frames | `ros2 launch fairino5_v6_moveit2_config static_virtual_joint_tfs.launch.py` |
| `warehouse_db.launch.py` | fairino5_v6_moveit2_config | Trajectory database (MongoDB) | `ros2 launch fairino5_v6_moveit2_config warehouse_db.launch.py` |
| `setup_assistant.launch.py` | fairino5_v6_moveit2_config | MoveIt Setup Assistant GUI | `ros2 launch fairino5_v6_moveit2_config setup_assistant.launch.py` |

---

## Switching to Real Robot Hardware

Per the [official Fairino ROS2 guide](https://manual.fairino.support/latest/ROSGuide/ros2guide.html):

### 1. Obtain the Fairino SDK
The `libfairino.so` library must be placed at:
```
ros2_sdk/src/fairino_hardware/libfairino/lib/libfairino.so.2.3.0
```

### 2. Build fairino_hardware
```bash
cd ~/ros2_ws
colcon build --packages-select fairino_msgs
source install/setup.bash
colcon build --packages-select fairino_hardware
source install/setup.bash
```

### 3. Switch the hardware plugin
Edit `fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro`:
```xml
<!-- Change FROM: -->
<plugin>mock_components/GenericSystem</plugin>

<!-- Change TO: -->
<plugin>fairino_hardware/FairinoHardwareInterface</plugin>
```

### 4. Launch the command server
```bash
ros2 run fairino_hardware ros2_cmd_server
```

### 5. Load hardware parameters
```bash
ros2 param load fr_command_server \
  ~/ros2_ws/src/fairino_hardware/fairino_remotecmdinterface_para.yaml
```

### 6. Monitor robot state
```bash
ros2 topic echo /nonrt_state_data
```

### 7. Send commands via rqt
```bash
rqt
# Navigate: Plugins → Services → Service Caller
# Select: /fairino_remote_command_service
# Enter command string (e.g. "MoveJ([0,0,0,0,0,0],30)") and click Call
```

### Command String Rules (from official docs)
- Format: `FunctionName(arg1,arg2,...)`
- Arguments: only letters, numbers, commas, minus signs
- No spaces or special characters (will cause errors)
- **GET commands** return strings; **other commands** return integers (0=error, 1=success)

---

## Official Documentation Reference

- **Fairino ROS2 Guide:** https://manual.fairino.support/latest/ROSGuide/ros2guide.html
- **MoveIt2 Integration:** https://manual.fairino.support/latest/ROSGuide/moveIt2.html

### Optional: MTC Demo Package
The official Fairino repo includes `fairino_mtc_demo` (Motion Task Control sample code) which is not present in this project. To add it, obtain it from the official Fairino repository and build with:
```bash
colcon build --packages-select fairino_mtc_demo
```

---

## Known Limitations

1. **`fairino_hardware` package** — Cannot build without proprietary `libfairino.so` SDK library. Contact Fairino for the SDK.
2. **PyBullet GUI** — Requires a display server (X11).
3. **MoveIt2 RViz** — Requires a display server.
4. **Joint order** — MoveIt2 joint_state_broadcaster may publish joints in non-sequential order (j1, j2, j4, j5, j3, j6). Use joint names, not array indices.
5. **`fairino_mtc_demo`** — Not included in this project. Available from the official Fairino repository.