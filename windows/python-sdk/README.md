# windows/python-sdk

Add-on Python modules for the Fairino FR5 on Windows. These extend the
Fairino Python SDK with simulation, planning, and gripper control.

## Prerequisites

```
pip install -r ../requirements.txt
```

Installs: `numpy`, `pybullet`, and optionally `inputs` (Xbox controller).

The Fairino SDK module (`Robot.py`) is included in this directory. Example
scripts add it to `sys.path` automatically — no separate install needed.
The SDK is only required when connecting to the real robot; all simulation
functionality works without it.

## Files

| File | Purpose |
|------|---------|
| `fr5_robot_interface.py` | Unified sim/real interface — wraps PyBullet and SDK RPC |
| `robot_controller.py` | Xbox controller teleoperation (class: `RobotController`) |
| `trajectory_replayer.py` | Record and replay joint trajectories from CSV |
| `rrt_planner.py` | Standalone RRT-Connect path planner for PyBullet |
| `gripper_controller.py` | AG-95 gripper state machine for simulation |
| `trajectory_interpolator.py` | Min-jerk trajectory interpolation |
| `cartesian_planner.py` | IK-based Cartesian path planning |
| `workspace_boundaries.py` | Joint + Cartesian limits, gripper-aware (`GRIPPER_OFFSET = 0.28`) |
| `live_viewer.py` | Real-time PyBullet mirror of real FR5 joints |

## URDF

```
urdf/
├── fairino5_v6.urdf               Base FR5 (no gripper)
├── fairino5_v6_with_ag95.urdf     FR5 + AG-95 gripper (recommended for sim)
├── fairino5_v6_with_gripper.urdf  FR5 + generic gripper
└── meshes/
    ├── base_link.STL              FR5 arm mesh files (7 total)
    ├── shoulder_link.STL
    ├── upperarm_link.STL
    ├── forearm_link.STL
    ├── wrist1_link.STL
    ├── wrist2_link.STL
    ├── wrist3_link.STL
    └── ag95/                      AG-95 gripper meshes (5 total)
        ├── base_link.STL
        ├── crank_Link.STL
        ├── distal_phalanx_Link.STL
        ├── proximal_phalanx_Link.STL
        └── rod_Link.STL
```

The URDF files reference meshes via `package://` paths. At runtime, scripts
patch these to absolute filesystem paths so PyBullet can load them. Do not
rename or move the `meshes/` directory relative to the URDF files.

## Usage

### Unified Robot Interface

```python
import sys
sys.path.insert(0, 'path/to/python-sdk')
from fr5_robot_interface import FR5RobotInterface

# Simulation (no robot needed)
with FR5RobotInterface(mode='simulation', gui=True) as robot:
    robot.move_j([0, -70, 90, -110, -90, 30])
    state = robot.get_state()
    print(state['joint_positions'])
    robot.open_gripper()

# Real robot
with FR5RobotInterface(mode='real', robot_ip='192.168.58.2') as robot:
    robot.move_j([0, -70, 90, -110, -90, 30], velocity=20)
    robot.close_gripper()
```

### RRT Planner

```python
from rrt_planner import RRTPlanner

planner = RRTPlanner(
    robot_id=robot_id,
    arm_joint_indices=arm_joints,
    joint_limits=joint_limits,
    obstacle_ids=[table_id, box_id],
)
path = planner.rrt_connect(start_rad, goal_rad)
smooth = planner.shortcut_path(path)
```

### Workspace Boundaries

```python
from workspace_boundaries import WorkspaceBoundary

# With gripper (default)
valid, msg = WorkspaceBoundary.check_cartesian_position([0.8, 0, 0.5])

# Without gripper
valid, msg = WorkspaceBoundary.check_cartesian_position([0.8, 0, 0.5], with_gripper=False)

print(WorkspaceBoundary.get_workspace_info())
```

### Live Viewer

```
python live_viewer.py 192.168.58.2
```

Reads joints from real FR5 at ~10 Hz, displays in PyBullet 3D view.
