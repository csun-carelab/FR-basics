# Simulation Mode

This mode simulates the FR5 robot using PyBullet for testing and development without requiring the physical robot. The simulation includes RRT-Connect path planning for collision-free motion.

## Setup

1. Install dependencies:
   ```bash
   pip install -r ../requirements.txt
   ```

2. Ensure PyBullet GUI can open (may require display setup on headless systems).

## Running the Example

```bash
python first_example.py
```

This will:
- Open PyBullet GUI with simulation environment (table, obstacle, target block)
- Load the FR5 robot model on the table
- Demonstrate RRT-Connect path planning around obstacles
- Execute pick-and-place style motion sequence:
  1. Move to target position (above blue block)
  2. Move to place position (other side of red cylinder obstacle)
  3. Return to home configuration

## Features

- **RRT-Connect Path Planning**: Bidirectional rapidly-exploring random tree algorithm
- **Collision Avoidance**: Automatically plans paths that avoid obstacles
- **Path Smoothing**: Shortcutting and interpolation for smooth motion
- **Environment Setup**: Table, target block (blue), obstacle cylinder (red)
- **Inverse Kinematics**: Computes joint angles for end-effector positions
- **Realistic robot dynamics simulation**
- **Visual feedback via PyBullet GUI**
- **Safe testing environment**

## Customization

- Modify `URDF_PATH` in `first_example.py` to use different robot models
- Adjust `GUI` variable to run headless (False) or with visualization (True)
- Modify obstacle and target positions in `RobotSim.__init__`
- Tune RRT-Connect parameters: `step_size`, `max_iters`, `goal_bias`

## Algorithm Details

The simulation follows the trajectory planning approach from `Final_trajectory_planning.py`:

1. **Environment Setup**: Creates simulation with table, obstacles, and target
2. **RRT-Connect Planning**: Builds bidirectional trees from start and goal
3. **Path Shortcutting**: Removes unnecessary waypoints
4. **Path Interpolation**: Adds intermediate points for smooth execution
5. **Motion Execution**: Follows planned path with position control

## Requirements

- PyBullet with GUI support
- NumPy
- Compatible graphics drivers for GUI