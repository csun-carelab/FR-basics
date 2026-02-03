# FR5 Robot Documentation

Complete documentation for the FR5 robot Python SDK and simulation environment.

## Documentation Structure

### 📘 [TUTORIAL.md](TUTORIAL.md)
**Complete tutorial from basics to advanced features**

Perfect for learning the FR5 SDK step-by-step:
- Getting started with your first program
- Basic motion control (MoveJ)
- Gripper control (DH AG-95)
- Workspace management and safety
- Cartesian planning (MoveL)
- Trajectory recording and playback
- ROS2 integration
- Advanced topics and common patterns

**Start here if you're new to the FR5 SDK!**

---

### 📗 [API_REFERENCE.md](API_REFERENCE.md)
**Complete API reference for all classes and methods**

Detailed documentation covering:
- `FR5RobotInterface` - Main robot control interface
- `TrajectoryInterpolator` - Smooth trajectory generation
- `WorkspaceBoundary` - Workspace validation
- `GripperController` - DH AG-95 gripper control
- `CartesianPlanner` - Cartesian path planning
- `TrajectoryRecorder` - Trajectory recording/playback
- ROS2 Services - Service and topic reference

**Use this as your reference while coding!**

---

## Quick Links

### For Beginners
1. Read [TUTORIAL.md](TUTORIAL.md) - Getting Started section
2. Run examples in `/home/cflores18/FR-basics/examples/`
3. Try modifying example scripts
4. Refer to [API_REFERENCE.md](API_REFERENCE.md) when needed

### For Developers
1. Review [API_REFERENCE.md](API_REFERENCE.md) for complete API
2. Check [IMPLEMENTATION_SUMMARY.md](../IMPLEMENTATION_SUMMARY.md) for architecture
3. Run test suite: `pytest /home/cflores18/FR-basics/tests/`
4. Build your application using SDK components

### For Integration
1. See [TUTORIAL.md](TUTORIAL.md) - ROS2 Integration section
2. Review ROS2 services in [API_REFERENCE.md](API_REFERENCE.md)
3. Check `/home/cflores18/FR-basics/examples/example_ros2_services.py`
4. Use tmux launch script for multi-terminal view

---

## Additional Resources

### Implementation Documentation
- **[IMPLEMENTATION_SUMMARY.md](../IMPLEMENTATION_SUMMARY.md)** - Complete implementation details
- Covers all features, architecture, and design decisions
- Lists all files created and modified
- Performance improvements and benchmarks

### Example Scripts
Located in `/home/cflores18/FR-basics/examples/`:
- `example_smooth_motion.py` - Smooth motion with minimum-jerk trajectories
- `example_workspace_boundaries.py` - Workspace validation
- `example_gripper_control.py` - DH AG-95 gripper control
- `example_cartesian_planning.py` - Cartesian path planning
- `example_trajectory_recording.py` - Recording and playback
- `example_ros2_services.py` - ROS2 service integration

### Test Suite
Located in `/home/cflores18/FR-basics/tests/`:
- `tests/unit/` - Unit tests for individual components
- `tests/integration/` - Integration tests for full system
- Run with: `pytest tests/ -v`

### Python SDK
Located in `/home/cflores18/FR-basics/python_sdk/`:
- `fr5_robot_interface.py` - Main interface
- `simulation/` - Simulation components
- `real_robot/fairino/` - Real robot SDK (fairino)

### ROS2 Workspace
Located in `/home/cflores18/fr5_ws/`:
- `src/fr5_pybullet_sim/` - ROS2 simulation node
- `scripts/launch_with_tmux.sh` - Multi-pane launch script
- 11 ROS2 packages for complete integration

---

## Documentation Coverage

### Topics Covered

✅ **Robot Control**
- Joint space motion (MoveJ)
- Cartesian space motion (MoveL)
- Smooth trajectory generation
- Collision detection

✅ **Gripper Operations**
- Open/close control
- Precise position control (0-95mm)
- Force control (0-45N per finger)
- Contact detection

✅ **Workspace Management**
- Joint limit validation
- Cartesian envelope checking
- Position clamping
- Safety boundaries

✅ **Path Planning**
- Cartesian pose interpolation
- IK-based path generation
- Path smoothness validation
- Collision-free planning

✅ **Trajectory Tools**
- Recording during motion
- Save/load trajectories
- Playback recorded motions
- Trajectory analysis

✅ **ROS2 Integration**
- SetJointPosition service
- GripperControl service
- /joint_states topic
- /tcp_pose topic

✅ **Development Tools**
- Python SDK usage
- Testing framework
- Example scripts
- API reference

---

## Getting Help

### Documentation Order (Recommended)

1. **Tutorial First** → [TUTORIAL.md](TUTORIAL.md)
   - Learn concepts step-by-step
   - Run examples as you read
   - Build understanding gradually

2. **API Reference** → [API_REFERENCE.md](API_REFERENCE.md)
   - Look up specific functions
   - Check parameter details
   - Find exact syntax

3. **Examples** → `/home/cflores18/FR-basics/examples/`
   - See working code
   - Understand usage patterns
   - Copy and modify for your needs

4. **Tests** → `/home/cflores18/FR-basics/tests/`
   - See edge cases
   - Understand validation
   - Check expected behavior

### Troubleshooting

**Problem:** Don't know where to start
→ Read [TUTORIAL.md](TUTORIAL.md) from the beginning

**Problem:** Need specific function details
→ Search [API_REFERENCE.md](API_REFERENCE.md)

**Problem:** Code not working
→ Check examples in `/examples/` directory
→ Run tests: `pytest tests/ -v`

**Problem:** Understanding architecture
→ Read [IMPLEMENTATION_SUMMARY.md](../IMPLEMENTATION_SUMMARY.md)

**Problem:** ROS2 integration issues
→ See [TUTORIAL.md](TUTORIAL.md) ROS2 section
→ Check `/examples/example_ros2_services.py`

---

## Contributing

### Documentation Standards

When adding documentation:
1. Update relevant sections in TUTORIAL.md and API_REFERENCE.md
2. Add working examples to `/examples/` directory
3. Include code comments and docstrings
4. Add test cases demonstrating usage
5. Update this README if adding new doc files

### Code Examples

All code examples should:
- Be runnable without modification
- Include necessary imports
- Show expected output
- Handle errors gracefully
- Follow Python best practices

---

## Version Information

**Current Version:** 1.0

**Python SDK:** Complete
- FR5RobotInterface (simulation + real robot)
- Smooth motion with minimum-jerk trajectories
- DH AG-95 gripper integration
- Workspace boundary validation
- Cartesian path planning
- Trajectory recording/playback

**ROS2 Integration:** Complete
- SetJointPosition service
- GripperControl service
- Joint state publishing
- TCP pose publishing
- tmux multi-pane visualization

**Testing:** Complete
- 19 unit tests (all passing)
- 11 integration tests
- pytest framework with markers
- Continuous validation

---

## License

Documentation for FR5 robot Python SDK and simulation environment.
For license information, see project root.

---

**Happy Learning! 🤖📚**

For questions or issues, refer to:
- [TUTORIAL.md](TUTORIAL.md) for learning
- [API_REFERENCE.md](API_REFERENCE.md) for reference
- `/examples/` for working code
- `/tests/` for validation
