# FR5 Basics Organized

Add-on library for the Fairino FR5 6-DOF collaborative robot (CSUN CARE Lab).

Provides PyBullet simulation, path planning, gripper control, and example scripts
for both Windows (standalone Python) and Linux (ROS2).

## Structure

```
FR5_basics_organized/
├── windows/           Python SDK add-ons and examples for Windows
│   ├── python-sdk/    Core modules, URDF models, mesh files
│   └── examples/      basic/ and advanced/ example scripts
└── ros-linux/         ROS2 add-ons and examples for Ubuntu/Linux
    ├── ros2-sdk/      ROS2 packages, planners, sim node
    ├── examples/      basic/ and advanced/ ROS2 example scripts
    └── config/        MoveIt2 configuration
```

## Hardware

| Item | Spec |
|------|------|
| Robot | Fairino FR5, 6-DOF |
| Reach (bare) | 922mm |
| Gripper | DH Robotics AG-95 (+280mm) |
| Effective reach | ~1.1m (conservative) |
| Default robot IP | 192.168.58.2 |

---

## Prerequisites

### Windows

**Python 3.10+** is required. Install dependencies:

```
cd windows
pip install -r requirements.txt
```

This installs:
- `numpy` — array math, trajectory computation
- `pybullet` — physics simulation and 3D visualization
- `inputs` — Xbox controller input (only needed for teleoperation examples)

**Fairino Python SDK** — The robot SDK module (`Robot.py`) is included in
`windows/python-sdk/`. The example scripts add this directory to `sys.path`
automatically. You do **not** need to `pip install fairino` separately.
The SDK is only needed when connecting to the real robot; simulation works
without it.

### Linux / ROS2

**ROS2 Humble** (or newer) must be installed and sourced. Additionally:

```bash
# Python packages (inside your ROS2 environment)
pip install pybullet numpy

# ROS2 packages (via apt)
sudo apt install ros-humble-moveit   # optional, for MoveIt2 demos
```

The Fairino ROS2 driver and `fairino_msgs` must be installed separately
per Fairino's documentation.

See [ros-linux/README.md](ros-linux/README.md) for build and launch instructions.

---

## Quick Start — Windows

### Simulation (no robot needed)

```
pip install -r windows/requirements.txt
python windows/examples/basic/01_first_example.py
```

This opens a PyBullet window showing the FR5 at its home position.

### Real Robot

```
python windows/examples/basic/01_first_example.py --real --ip 192.168.58.2
```

Requires network access to the FR5 controller at the specified IP.

### Live Viewer (mirrors real robot in PyBullet)

```
python windows/python-sdk/live_viewer.py 192.168.58.2
```

### Xbox Controller Teleoperation

```
pip install inputs
python windows/examples/basic/06_xbox_controller.py
```

---

## Quick Start — Linux / ROS2

### PyBullet Simulation

```bash
# Terminal 1: start sim node
ros2 launch fr5_pybullet_sim fr5_sim.launch.py

# Terminal 2: run examples
python ros-linux/examples/basic/01_first_example.py
```

### MoveIt2

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
python ros-linux/examples/advanced/moveit2_demo.py
```

---

## Windows Examples

### Basic

| Script | Description | Sim | Real |
|--------|-------------|-----|------|
| `01_first_example.py` | Connect, move to home, read joints | Yes | Yes |
| `02_move_joint.py` | Move to a target joint position | No | Yes |
| `03_actuate_gripper.py` | Open/close AG-95 gripper | No | Yes |
| `04_kinematics.py` | Forward/inverse kinematics | No | Yes |
| `05_torque_control.py` | Torque-mode servo control | No | Yes |
| `06_xbox_controller.py` | Xbox gamepad teleoperation | No | Yes |
| `07_trajectory_replay.py` | Record and replay trajectories | Yes | Yes |
| `08_basic_commands.py` | SDK command reference | No | Yes |
| `09_motion_commands.py` | Motion command reference | No | Yes |
| `10_angular_speed.py` | Angular velocity control | No | Yes |
| `11_status_check.py` | Robot status queries | No | Yes |

### Advanced (all use `fr5_robot_interface.py`)

| Script | Description |
|--------|-------------|
| `pick_and_place.py` | Full pick-and-place pipeline |
| `rrt_pick_and_place.py` | RRT-Connect collision-free pick-and-place |
| `cartesian_planning.py` | IK-based Cartesian path planning |
| `gripper_control_advanced.py` | Gripper state machine and force control |
| `smooth_motion.py` | Min-jerk trajectory interpolation |
| `trajectory_recording.py` | CSV trajectory record and replay |
| `workspace_boundaries_demo.py` | Joint and Cartesian limit checking |
| `io_commands.py` | Digital/analog I/O reference |
| `set_commands.py` | Configuration command reference |
| `peripherals.py` | Peripheral device reference |

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'pybullet'`**
Run `pip install pybullet`. On Windows this compiles from source and requires
a C++ compiler (Visual Studio Build Tools).

**`pybullet.error: Cannot load URDF file`**
The URDF files use `package://` mesh paths that must be patched to absolute
paths at runtime. The scripts handle this automatically. If you moved the
repo, make sure the `urdf/meshes/` directory (including `meshes/ag95/`) is
intact relative to the URDF files.

**`ModuleNotFoundError: No module named 'Robot'`**
This is the Fairino SDK. It is only needed for real robot connections.
Simulation examples work without it. The SDK module is at
`windows/python-sdk/Robot.py` and is added to `sys.path` by each script.

**`ModuleNotFoundError: No module named 'inputs'`**
Only needed for Xbox controller scripts. Run `pip install inputs`.

**Real robot connection fails**
Ensure your PC is on the same subnet as the FR5 (default: 192.168.58.x)
and the robot controller is powered on.
