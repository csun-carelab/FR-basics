# ros-linux/ros2-sdk

ROS2 add-on modules and packages. These supplement the installed Fairino ROS2 SDK.

## Python Modules

### rrt_planner.py

Standalone RRT-Connect path planner. Same algorithm as `windows/python-sdk/rrt_planner.py`
but with optional `rclpy` logger support.

```python
from rrt_planner import RRTPlanner

# With ROS2 logging (pass self.get_logger() from a Node)
planner = RRTPlanner(robot_id, arm_joints, joint_limits, obstacles,
                     logger=self.get_logger())

# Or without (falls back to print)
planner = RRTPlanner(robot_id, arm_joints, joint_limits, obstacles)

path = planner.rrt_connect(start_rad, goal_rad)
smooth = planner.shortcut_path(path)
```

### workspace_boundaries.py

Gripper-aware workspace limits. Note: `visualize_workspace()` raises `NotImplementedError`
in ROS2 — use RViz markers for visualization.

```python
from workspace_boundaries import WorkspaceBoundary

valid, msg = WorkspaceBoundary.check_cartesian_position([0.8, 0, 0.5])
print(WorkspaceBoundary.get_workspace_info())
```

### trajectory_interpolator.py

Min-jerk trajectory interpolation between waypoints. Used by the sim node internally.

## fr5_pybullet_sim Package

ROS2 Python package providing a PyBullet physics simulation node for the FR5.

```
fr5_pybullet_sim/
├── fr5_pybullet_sim/
│   ├── __init__.py
│   └── sim_node.py        FR5PyBulletSim node
├── launch/
│   └── fr5_sim.launch.py
├── setup.py, setup.cfg, package.xml
└── resource/
```

**Build:**
```bash
cp -r fr5_pybullet_sim /path/to/ros2_ws/src/
cd /path/to/ros2_ws
colcon build --packages-select fr5_pybullet_sim
source install/setup.bash
```

**Launch parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gui` | `true` | Show PyBullet GUI |
| `urdf_name` | `fairino5_v6.urdf` | Which URDF to load |
| `sim_rate` | `120.0` | Simulation Hz |

## dh_gripper/ag95_description

AG-95 gripper URDF and mesh files for ROS2 packages.

```
dh_gripper/ag95_description/
├── urdf/dh_robotics_ag95.urdf
└── meshes/
    ├── base_link.STL
    ├── crank_Link.STL
    ├── distal_phalanx_Link.STL
    ├── proximal_phalanx_Link.STL
    └── rod_Link.STL
```

## urdf/

Standalone URDF files for reference or direct PyBullet loading.

```
urdf/
├── fairino5_v6.urdf               Base FR5
├── fairino5_v6_with_ag95.urdf     FR5 + AG-95 gripper
└── fairino5_v6/                   STL meshes (7 links)
```
