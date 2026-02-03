# ROS2 Real Robot Mode

This mode provides ROS2-based control of the physical FR5 robot.

## Setup

1. Ensure ROS2 is installed and the workspace is built:
   ```bash
   cd ../..
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Set robot IP (default: 192.168.58.2). Modify launch files if needed.

3. Enable the robot via teach pendant.

## Running the Example

```bash
python first_example.py
```

This will:
- Launch ROS2 nodes for robot control
- Read current joint positions via ROS2 services
- Move the first joint by +15 degrees
- Shutdown ROS2 nodes

## Launch Files

- `launch/`: Contains ROS2 launch files for robot bringup

## Configuration

- `config/`: Robot configuration files (IP, etc.)

## Safety Notes

- Monitor robot during operation
- Ensure robot is enabled and safe
- Check network connectivity

## Troubleshooting

- Verify ROS2 installation
- Check `colcon build` succeeds
- Ensure robot IP is correct