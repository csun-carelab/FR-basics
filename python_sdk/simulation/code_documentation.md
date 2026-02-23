# First Example Robot Simulation - Code Documentation

## Overview
This file implements a complete pick-and-place robot simulation using PyBullet with RRT-Connect path planning. The code simulates a FR5 robot with an AG-95 gripper performing collision-free motion planning.

## File Structure & Responsibilities

### 1. Imports and Dependencies
```python
import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import os
import random
from trajectory_interpolator import TrajectoryInterpolator
```
**Responsibility**: Load required libraries for physics simulation, math operations, and trajectory generation.

### 2. Global Configuration
```python
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "urdf", "fairino5_v6_with_ag95.urdf")
GUI = True
DT = 1/120.0
COLLISION_MARGIN = 0.02
```
**Responsibility**: Define simulation parameters, file paths, and physical constants.

---

## Classes

### 3. RobotStatePkg Class
```python
class RobotStatePkg:
    def __init__(self, robot_sim):
        self._robot_sim = robot_sim
    
    @property
    def jt_cur_pos(self):
        return self._robot_sim._get_current_joint_positions()
```
**Responsibility**: Wrapper for robot state information. Provides current joint positions in degrees.

### 4. RobotSim Class
Main simulation class handling all robot operations.

#### 4.1 URDF Path Patching
```python
@staticmethod
def _patch_urdf_for_pybullet(urdf_path):
```
**Responsibility**: Fix mesh file paths in URDF for PyBullet compatibility. Converts `package://` paths to absolute file system paths.

#### 4.2 Initialization
```python
def __init__(self, urdf_path=None, gui=True):
```
**Responsibility**: 
- Initialize PyBullet physics client
- Load ground plane and table environment
- Load and configure robot model
- Detect and categorize all robot joints
- Set up workspace objects (target block, obstacle)
- Initialize home configuration and gripper state

#### 4.3 Joint Detection
```python
for j in range(num_joints):
    info = p.getJointInfo(self.robot_id, j)
    # Categorize joints as arm, gripper, or end-effector
```
**Responsibility**: Scan robot URDF to identify controllable joints, gripper joints, and end-effector frames.

#### 4.4 Environment Setup
```python
# Table setup
self.table_xyz = [0.5, 0, 0]
self.table = p.loadURDF("table/table.urdf", self.table_xyz)

# Target block
self.target_pos = np.array([0.5, -0.25, self.tabletop_z + 0.025])

# Obstacle
self.obstacle_pos = np.array([0.5, 0., self.tabletop_z + 0.15])
```
**Responsibility**: Create workspace objects for pick-and-place task.

#### 4.5 Home Configuration
```python
self.home_config = [
    math.radians(0.),      # j1: base rotation
    math.radians(60.),     # j2: shoulder 
    math.radians(100.),    # j3: elbow
    math.radians(110.),    # j4: wrist1
    math.radians(90.),     # j5: wrist2
    math.radians(0.)      # j6: wrist3
]
```
**Responsibility**: Define robot's rest/starting position with joints in opposite direction for better workspace clearance.

#### 4.6 Gripper Control Methods
```python
def _get_finger_position(self):
def _set_gripper_position(self, position, force=100):
def open_gripper(self, duration=1.0):
def close_gripper(self, duration=1.0):
```
**Responsibility**: 
- Read current gripper finger positions
- Control AG-95 gripper joints with mimic behavior
- Implement smooth opening/closing with minimum-jerk trajectories
- Detect object grasp through contact forces

#### 4.7 Robot Motion Methods
```python
def _get_current_joint_positions(self):
def _get_arm_q(self):
def _reset_to_home(self):
def _exec_to_q(self, q_target, duration=0.5, pos_gain=0.3, vel_gain=0.8, force=200):
```
**Responsibility**:
- Read current joint states
- Convert between degrees and radians
- Reset robot to home position smoothly
- Execute smooth joint motions with PID control

---

## 5. Path Planning System

### 5.1 Collision Detection
```python
def _in_collision(self, q, self_collisions=False, margin=None):
def _edge_collision_free(self, q_a, q_b, num_checks=30):
```
**Responsibility**:
- Check if joint configuration causes collisions
- Validate path segments between waypoints
- Handle self-collisions and obstacle collisions with safety margins

### 5.2 RRT-Connect Algorithm
```python
def rrt_connect(self, q_start, q_goal, step_size=0.12, max_iters=4000, goal_bias=0.20):
```
**Responsibility**:
- Implement bidirectional RRT-Connect path planning
- Grow trees from start and goal simultaneously
- Find collision-free paths in high-dimensional joint space
- Handle goal biasing for faster convergence

**Key Components**:
- `_sample_q()`: Generate random joint configurations
- `_steer()`: Move toward target with step size limit
- `_nearest()`: Find nearest node in tree
- `extend()`: Attempt to add node to tree
- `connect()`: Try to connect to target

### 5.3 Path Smoothing
```python
def shortcut_path(self, path, tries=250):
```
**Responsibility**: Post-process RRT path to remove unnecessary waypoints and create shorter, smoother trajectories.

### 5.4 Trajectory Interpolation
```python
trajectory = TrajectoryInterpolator.minimum_jerk(q_current, q_target, num_steps)
```
**Responsibility**: Generate smooth minimum-jerk trajectories between waypoints to eliminate jerky motions.

---

## 6. High-Level Robot Interface

### 6.1 Motion Primitives
```python
def MoveJ(self, joint_pos, tool=0, user=0, vel=100, acc=100, ovl=100):
```
**Responsibility**: 
- Primary motion interface mimicking industrial robot API
- Convert input degrees to radians
- Plan collision-free path using RRT-Connect
- Execute smooth trajectory with interpolation
- Handle planning failures with fallback direct motion

### 6.2 Task-Specific Motions
```python
def MoveToTarget(self):
def MoveToPlace(self):
def DescendToGrasp(self, target_z):
def LiftObject(self, lift_height=0.25):
def ReturnHome(self):
```
**Responsibility**:
- **MoveToTarget/Place**: Use inverse kinematics to reach positions above objects
- **DescendToGrasp**: Controlled vertical descent to grasp height
- **LiftObject**: Lift grasped object with collision avoidance
- **ReturnHome**: Navigate back to safe rest configuration

### 6.3 Inverse Kinematics
```python
theta = p.calculateInverseKinematics(
    self.robot_id, self.grasp_frame_index, target_position, ee_quat,
    solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
)
```
**Responsibility**: Compute joint angles to achieve desired end-effector pose using Damped Least Squares solver.

---

## 7. Utility Methods

### 7.1 Connection Management
```python
def CloseRPC(self):
    p.disconnect()
```
**Responsibility**: Clean shutdown of PyBullet simulation.

### 7.2 State Management
```python
self.robot_state_pkg = RobotStatePkg(self)
self.grasp_constraint = None
```
**Responsibility**: Maintain robot state and object grasping constraints.

---

## 8. Main Function

### 8.1 Simulation Setup
```python
def main():
    print("=" * 60)
    robot = RobotSim(gui=GUI)
    time.sleep(3)
```
**Responsibility**: Initialize simulation and display system information.

### 8.2 Pick-and-Place Sequence
```python
# 1. Move to target position
rtn = robot.MoveToTarget()

# 2. Descend to grasp
robot.DescendToGrasp(robot.target_pos[2] + 0.005)

# 3. Grasp object
robot.close_gripper()

# 4. Lift object
robot.LiftObject()

# 5. Move to place position
rtn = robot.MoveToPlace()

# 6. Release object
robot.open_gripper()

# 7. Return home
rtn = robot.ReturnHome()
```
**Responsibility**: Execute complete pick-and-place task demonstrating all system capabilities.

### 8.3 Error Handling
```python
try:
    # Robot operations
except Exception as e:
    import traceback
    print("=== ERROR CAUGHT ===")
    traceback.print_exc()
```
**Responsibility**: Catch and report errors gracefully, allowing simulation to continue running for debugging.

### 8.4 Simulation Loop
```python
if GUI:
    while True:
        p.stepSimulation()
        time.sleep(DT)
```
**Responsibility**: Keep simulation running for visualization until user closes window.

---

## Key Algorithms

### RRT-Connect Algorithm
1. Initialize two trees: one from start, one from goal
2. Sample random configurations with goal bias
3. Grow trees alternately using steering function
4. Attempt to connect trees when they're close
5. Extract path by backtracking through parent pointers

### Minimum-Jerk Trajectory
- Uses 5th-order polynomial: `s(t) = 10t³ - 15t⁴ + 6t⁵`
- Ensures zero velocity and acceleration at endpoints
- Creates smooth, natural-looking robot motions

### Collision Detection
- Tests configurations in PyBullet's physics engine
- Uses safety margins for obstacle avoidance
- Checks both self-collisions and environment collisions

---

## Dependencies

### External Files
- **trajectory_interpolator.py**: Provides smooth trajectory generation
- **fairino5_v6_with_ag95.urdf**: Robot and gripper model
- **table/table.urdf**: Environment furniture
- **STL mesh files**: Visual and collision geometry

### PyBullet Functions Used
- `p.connect()`: Initialize physics client
- `p.loadURDF()`: Load models from URDF files
- `p.calculateInverseKinematics()`: Compute joint angles for poses
- `p.getJointInfo()`: Query joint properties
- `p.getContactPoints()`: Detect collisions
- `p.createConstraint()`: Fix grasped objects
- `p.stepSimulation()`: Advance physics simulation

---

## Configuration Parameters

| Parameter | Value | Purpose |
|-----------|--------|---------|
| DT | 1/120.0 | Simulation timestep (120 Hz) |
| COLLISION_MARGIN | 0.02 | Safety margin for obstacles (2cm) |
| GUI | True | Enable visual simulation |
| RRT step_size | 0.12 | Max expansion per iteration |
| RRT max_iters | 4000 | Planning timeout |
| IK solver | DLS | Damped Least Squares method |
| Joint limits | ±2.8 rad | Default joint limits if unspecified |

---

## Usage Examples

```python
# Basic usage
robot = RobotSim(gui=True)
robot.MoveJ([0, -60, -100, -110, 90, 0])
robot.close_gripper()
robot.open_gripper()
robot.CloseRPC()

# Pick and place
robot.MoveToTarget()
robot.DescendToGrasp(0.05)
robot.close_gripper()
robot.LiftObject()
robot.MoveToPlace()
robot.open_gripper()
robot.ReturnHome()
```

This comprehensive simulation demonstrates industrial robot programming concepts including path planning, collision avoidance, and smooth motion control in a physics-based environment.