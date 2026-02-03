# Troubleshooting Guide

Common issues and solutions for FR5 robot integration.

## Python SDK Issues

### PyBullet GUI Not Opening
**Symptoms**: Simulation script fails or no window appears
**Solutions**:
- Install graphics drivers
- Run with `DISPLAY=:0` if headless
- Use `p.connect(p.DIRECT)` for headless operation

### Fairino SDK Import Error
**Symptoms**: `ImportError: No module named 'Robot'`
**Solutions**:
- `pip install fairino`
- Check Python path
- Verify SDK installation

### Robot Connection Failed
**Symptoms**: Connection timeout or RPC error
**Solutions**:
- Ping robot IP: `ping 192.168.58.2`
- Check robot power and network
- Verify IP in script matches robot

## ROS2 SDK Issues

### Colcon Build Fails
**Symptoms**: Build errors in packages
**Solutions**:
- Install missing deps: `sudo apt install ros-jazzy-rosidl-default-generators`
- Clean build: `rm -rf build/ install/ && colcon build`
- Check ROS2 version compatibility

### Launch Files Not Found
**Symptoms**: `ros2 launch` command fails
**Solutions**:
- Source workspace: `source install/setup.bash`
- Check package names: `ros2 pkg list`
- Verify launch file paths

### Service Calls Fail
**Symptoms**: Service not available or timeout
**Solutions**:
- Check nodes running: `ros2 node list`
- Verify services: `ros2 service list`
- Ensure robot bridge is active

### Joint State Topic Empty
**Symptoms**: No data on `/joint_states`
**Solutions**:
- Check state publisher node running
- Verify robot connection
- Debug with `ros2 topic echo /joint_states`

## General Issues

### Permission Errors
**Symptoms**: Access denied to files or devices
**Solutions**:
- Run with sudo if needed
- Check file permissions
- Add user to dialout group for serial

### Network Issues
**Symptoms**: Connection timeouts
**Solutions**:
- Check firewall settings
- Verify subnet configuration
- Use static IP addresses

### Performance Issues
**Symptoms**: Slow response or lag
**Solutions**:
- Check CPU usage
- Reduce simulation complexity
- Optimize network settings

## Debug Tools

### Python SDK
```bash
python -c "import Robot; print('SDK OK')"
python -c "import pybullet as p; print('PyBullet OK')"
```

### ROS2 SDK
```bash
ros2 doctor
ros2 pkg list | grep fr5
ros2 node list
ros2 topic list
ros2 service list
```

## Getting Help
- Check repository issues
- Review ROS2 documentation
- Contact Fairino support for hardware issues