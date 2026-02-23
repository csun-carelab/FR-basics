#!/usr/bin/env python3
"""
Example: Trajectory Recording and Playback

Demonstrates trajectory recording, saving, loading, and playback.
Useful for teaching by demonstration and trajectory optimization.
"""

import sys
import os
import time

# Add python_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python_sdk'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python_sdk/simulation'))

from fr5_robot_interface import FR5RobotInterface
from cartesian_planner import TrajectoryRecorder
import numpy as np


def main():
    print("=" * 70)
    print("FR5 Trajectory Recording Example")
    print("=" * 70)
    print("\nDemonstrating trajectory recording, saving, loading, and playback.\n")

    with FR5RobotInterface(mode='simulation', gui=True) as robot:
        print("✓ Robot connected in simulation mode\n")

        # Create trajectory recorder
        recorder = TrajectoryRecorder()

        # Example 1: Record a simple motion sequence
        print("1. Recording a Pick-and-Place Trajectory")
        print("-" * 70)

        trajectory_name = "pick_and_place_demo"
        print(f"   Starting recording: '{trajectory_name}'")
        recorder.start_recording(trajectory_name)

        # Define motion sequence
        waypoints = [
            ([0, 0, 0, 0, 0, 0], "Home position"),
            ([0, -30, 45, -60, 0, 0], "Approach position"),
            ([0, -40, 60, -80, 0, 0], "Pick position"),
            ([0, -30, 45, -60, 0, 0], "Lift position"),
            ([45, -30, 45, -60, 0, 45], "Move to place"),
            ([45, -40, 60, -80, 0, 45], "Place position"),
            ([45, -30, 45, -60, 0, 45], "Retract"),
            ([0, 0, 0, 0, 0, 0], "Return home"),
        ]

        print(f"   Recording {len(waypoints)} waypoints...\n")

        for i, (joints, description) in enumerate(waypoints):
            print(f"   Waypoint {i+1}: {description}")

            # Move to position
            valid, msg = robot.check_workspace_limits(joints)
            if valid:
                robot.move_j(joints, velocity=40)
                time.sleep(0.5)

                # Record current position
                current_joints = robot.get_joint_positions()
                recorder.record_waypoint(current_joints, timestamp=time.time())
                print(f"              {[f'{j:.1f}' for j in current_joints]}")
            else:
                print(f"   ⚠ Skipped: {msg}")

            time.sleep(0.3)

        # Stop recording
        num_recorded = recorder.stop_recording(trajectory_name)
        print(f"\n   ✓ Recording stopped: {num_recorded} waypoints captured\n")

        # Example 2: Save trajectory to file
        print("2. Saving Trajectory to Disk")
        print("-" * 70)

        trajectory_file = "/tmp/fr5_trajectory_demo.npy"
        recorder.save_trajectory(trajectory_name, trajectory_file)
        print(f"   ✓ Saved to: {trajectory_file}\n")

        # Example 3: List all recorded trajectories
        print("3. Listing All Recorded Trajectories")
        print("-" * 70)

        all_trajectories = recorder.list_trajectories()
        print(f"   Total trajectories: {len(all_trajectories)}")
        for traj_name in all_trajectories:
            traj = recorder.get_trajectory(traj_name)
            if traj:
                print(f"   • {traj_name}: {len(traj)} waypoints")
        print()

        # Example 4: Load trajectory from file
        print("4. Loading Trajectory from Disk")
        print("-" * 70)

        # Clear recorder and reload
        recorder.clear_all()
        print("   Cleared all trajectories from memory")

        loaded_name = recorder.load_trajectory(trajectory_file)
        print(f"   ✓ Loaded: '{loaded_name}'\n")

        # Example 5: Replay recorded trajectory
        print("5. Replaying Recorded Trajectory")
        print("-" * 70)

        trajectory = recorder.get_trajectory(loaded_name)
        if trajectory:
            print(f"   Playing back {len(trajectory)} waypoints...")
            print(f"   Speed: 50% of recording speed\n")

            for i, waypoint_joints in enumerate(trajectory):
                print(f"   Waypoint {i+1}/{len(trajectory)}: {[f'{j:.1f}' for j in waypoint_joints]}")

                valid, msg = robot.check_workspace_limits(waypoint_joints.tolist())
                if valid:
                    result = robot.move_j(waypoint_joints.tolist(), velocity=50)
                    if result == 0:
                        time.sleep(0.2)
                    else:
                        print(f"      ⚠ Motion failed")
                else:
                    print(f"      ⚠ Skipped: {msg}")

            print(f"\n   ✓ Playback completed\n")
        else:
            print("   ✗ Trajectory not found\n")

        # Example 6: Record another trajectory
        print("6. Recording a Second Trajectory")
        print("-" * 70)

        scan_trajectory = "scanning_motion"
        print(f"   Recording: '{scan_trajectory}'")
        recorder.start_recording(scan_trajectory)

        # Scanning motion (side to side)
        scan_positions = [
            [-45, -20, 30, -45, 0, 0],
            [-30, -20, 30, -45, 0, 0],
            [-15, -20, 30, -45, 0, 0],
            [0, -20, 30, -45, 0, 0],
            [15, -20, 30, -45, 0, 0],
            [30, -20, 30, -45, 0, 0],
            [45, -20, 30, -45, 0, 0],
        ]

        for joints in scan_positions:
            valid, msg = robot.check_workspace_limits(joints)
            if valid:
                robot.move_j(joints, velocity=30)
                time.sleep(0.3)
                current_joints = robot.get_joint_positions()
                recorder.record_waypoint(current_joints)

        num_scan = recorder.stop_recording(scan_trajectory)
        print(f"   ✓ Recorded {num_scan} waypoints\n")

        # Example 7: Compare trajectories
        print("7. Trajectory Comparison")
        print("-" * 70)

        all_trajectories = recorder.list_trajectories()
        for traj_name in all_trajectories:
            traj = recorder.get_trajectory(traj_name)
            if traj:
                # Calculate trajectory statistics
                path_length = 0
                for i in range(len(traj) - 1):
                    diff = np.linalg.norm(traj[i+1] - traj[i])
                    path_length += diff

                print(f"\n   Trajectory: {traj_name}")
                print(f"   • Waypoints: {len(traj)}")
                print(f"   • Path length: {path_length:.2f} rad")
                print(f"   • Start: {[f'{j:.1f}' for j in traj[0]]}")
                print(f"   • End:   {[f'{j:.1f}' for j in traj[-1]]}")

        print("\n" + "=" * 70)
        print("Trajectory Recording Features:")
        print("  ✓ Record waypoints during motion")
        print("  ✓ Save/load trajectories to disk (.npy format)")
        print("  ✓ List and manage multiple trajectories")
        print("  ✓ Replay recorded motions")
        print("  ✓ Teaching by demonstration")
        print("  ✓ Trajectory analysis and optimization")
        print("=" * 70)
        print("\nUsage in code:")
        print("  from cartesian_planner import TrajectoryRecorder")
        print("  recorder = TrajectoryRecorder()")
        print("  recorder.start_recording('my_trajectory')")
        print("  recorder.record_waypoint(joints)")
        print("  recorder.stop_recording('my_trajectory')")
        print("  recorder.save_trajectory('my_trajectory', 'file.npy')")
        print("=" * 70)


if __name__ == '__main__':
    main()
