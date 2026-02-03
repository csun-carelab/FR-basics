#!/usr/bin/env python3
"""
FR5 Robot ROS 2 Simulation - First Example

This example demonstrates controlling the FR5 robot simulation through ROS 2 services.

There are TWO ways to run this simulation:

METHOD 1: Standalone (No ROS 2 required)
-----------------------------------------
Simply run this script directly - it will start the PyBullet simulation
with RRT-Connect path planning built-in:

    python3 first_example.py

For headless environments (WSL, servers without display):

    python3 first_example.py --headless

METHOD 2: ROS 2 Node (Requires ROS 2 workspace built)
------------------------------------------------------
1. Build the workspace:
    cd /home/cflores18/FR-basics/ros2_sdk
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select fr5_pybullet_sim

2. Source the workspace:
    source install/setup.bash

3. Run the simulation node:
    ros2 run fr5_pybullet_sim sim_node

4. In another terminal, call services:
    ros2 service call /fr5_sim/go_home std_srvs/srv/Trigger
    ros2 service call /fr5_sim/go_to_target std_srvs/srv/Trigger
    ros2 service call /fr5_sim/go_to_place std_srvs/srv/Trigger

Or run this script as a ROS 2 client:
    python3 first_example.py --ros2
"""

import sys
import os
import argparse

# Check if running as ROS 2 client
def run_ros2_client():
    """Run as ROS 2 service client"""
    try:
        import rclpy
        from rclpy.node import Node
        from std_srvs.srv import Trigger
    except ImportError:
        print("ERROR: ROS 2 Python packages not found.")
        print("Make sure you have sourced ROS 2 setup:")
        print("  source /opt/ros/jazzy/setup.bash")
        print("  source /path/to/ros2_sdk/install/setup.bash")
        sys.exit(1)

    class FR5SimClient(Node):
        def __init__(self):
            super().__init__('fr5_sim_client')
            self.home_client = self.create_client(Trigger, '/fr5_sim/go_home')
            self.target_client = self.create_client(Trigger, '/fr5_sim/go_to_target')
            self.place_client = self.create_client(Trigger, '/fr5_sim/go_to_place')

        def wait_for_services(self, timeout=5.0):
            """Wait for simulation services to be available"""
            self.get_logger().info("Waiting for simulation services...")
            services = [
                (self.home_client, '/fr5_sim/go_home'),
                (self.target_client, '/fr5_sim/go_to_target'),
                (self.place_client, '/fr5_sim/go_to_place'),
            ]
            for client, name in services:
                if not client.wait_for_service(timeout_sec=timeout):
                    self.get_logger().error(f"Service {name} not available!")
                    return False
            self.get_logger().info("All services available!")
            return True

        def call_service(self, client, name):
            """Call a trigger service and wait for response"""
            self.get_logger().info(f"Calling {name}...")
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
            if future.result() is not None:
                result = future.result()
                self.get_logger().info(f"  Result: {result.message}")
                return result.success
            else:
                self.get_logger().error(f"  Service call failed!")
                return False

    rclpy.init()
    client = FR5SimClient()

    try:
        if not client.wait_for_services():
            print("\nMake sure the simulation node is running:")
            print("  ros2 run fr5_pybullet_sim sim_node")
            sys.exit(1)

        print("\n" + "=" * 60)
        print("FR5 Simulation Client - Calling motion services")
        print("=" * 60)

        # Go to target
        print("\n--- Moving to target position (above blue block) ---")
        client.call_service(client.target_client, '/fr5_sim/go_to_target')

        # Go to place
        print("\n--- Moving to place position (other side of obstacle) ---")
        client.call_service(client.place_client, '/fr5_sim/go_to_place')

        # Return home
        print("\n--- Returning to home position ---")
        client.call_service(client.home_client, '/fr5_sim/go_home')

        print("\n" + "=" * 60)
        print("Demo complete!")
        print("=" * 60)

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


def run_standalone(headless=False):
    """Run standalone PyBullet simulation without ROS 2"""
    import pybullet as p
    import pybullet_data
    import time
    import math
    import numpy as np
    import random
    import re
    import tempfile
    from trajectory_interpolator import TrajectoryInterpolator

    # Settings
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    # Use URDF with fully articulated AG-95 gripper
    URDF_PATH = os.path.join(SCRIPT_DIR, "urdf", "fairino5_v6_with_ag95.urdf")

    def patch_urdf_for_pybullet(urdf_path):
        """Patch URDF file to replace package:// mesh paths with absolute paths"""
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

        with open(urdf_path, 'r') as f:
            urdf_content = f.read()

        # Replace package://fairino_description/meshes/fairino5_v6/ with absolute path
        meshes_dir = os.path.join(urdf_dir, 'fairino5_v6')
        urdf_content = re.sub(
            r'package://fairino_description/meshes/fairino5_v6',
            meshes_dir,
            urdf_content
        )

        # Replace package://ag95_meshes/ with absolute path to AG-95 gripper meshes
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(urdf_dir)))
        ag95_meshes_dir = os.path.join(project_root, 'meshes', 'gripper', 'ag95')
        urdf_content = re.sub(
            r'package://ag95_meshes',
            ag95_meshes_dir,
            urdf_content
        )

        # Also handle old gripper path for backwards compatibility
        gripper_meshes_dir = os.path.join(project_root, 'meshes', 'gripper')
        urdf_content = re.sub(
            r'package://fr_basics/meshes/gripper',
            gripper_meshes_dir,
            urdf_content
        )

        # Create temporary patched URDF file
        temp_fd, temp_path = tempfile.mkstemp(suffix='.urdf', text=True)
        with os.fdopen(temp_fd, 'w') as f:
            f.write(urdf_content)

        return temp_path
    GUI = not headless
    DT = 1/120.0

    print("=" * 60)
    print("FR5 Robot Standalone Simulation with RRT-Connect")
    print("=" * 60)

    # PyBullet setup
    p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(DT)
    p.resetDebugVisualizerCamera(2.0, 45, -30, [0.5, 0, 0.5])
    p.setGravity(0, 0, -9.81)

    # Load environment
    p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf", [0.5, 0, 0])
    tabletop_z = p.getAABB(table)[1][2]

    # Load robot
    if not os.path.exists(URDF_PATH):
        print(f"ERROR: URDF not found at {URDF_PATH}")
        sys.exit(1)

    # Patch URDF to fix mesh paths for PyBullet
    patched_urdf_path = patch_urdf_for_pybullet(URDF_PATH)

    base_orientation = p.getQuaternionFromEuler([0, 0, math.pi])
    robot = p.loadURDF(patched_urdf_path, [0, 0, tabletop_z + 0.001],
                       base_orientation, useFixedBase=True)

    # Find arm joints, gripper joints, and grasp frame
    arm_joints = []
    gripper_joints = {}  # Dict to store AG-95 gripper joints by name
    end_effector = None
    grasp_frame = None
    num_joints = p.getNumJoints(robot)

    print(f"Scanning {num_joints} joints...")
    for j in range(num_joints):
        info = p.getJointInfo(robot, j)
        joint_type = info[2]
        joint_name = info[1].decode()
        link_name = info[12].decode()

        # Arm joints (j1-j6, revolute, excluding gripper joints)
        if joint_type == p.JOINT_REVOLUTE and "gripper" not in joint_name.lower() and "finger" not in joint_name.lower():
            arm_joints.append(j)
            print(f"  Arm joint {j}: {joint_name} -> {link_name}")

        # AG-95 gripper joints (revolute)
        if "gripper" in joint_name.lower() and joint_type == p.JOINT_REVOLUTE:
            gripper_joints[joint_name] = j
            print(f"  Gripper joint {j}: {joint_name}")

        # End effector / grasp frame
        if "wrist3" in link_name.lower():
            end_effector = j
        if "grasp" in link_name.lower():
            grasp_frame = j

    if end_effector is None and arm_joints:
        end_effector = arm_joints[-1]
    if grasp_frame is None:
        grasp_frame = end_effector

    print(f"Found {len(arm_joints)} arm joints, {len(gripper_joints)} gripper joints")
    print(f"End effector index: {end_effector}, Grasp frame index: {grasp_frame}")

    # AG-95 gripper mimic multipliers
    gripper_mimic_config = {
        'gripper_finger1_joint': 1.0,
        'gripper_finger2_joint': 1.0,
        'gripper_finger1_inner_knuckle_joint': 1.49462955,
        'gripper_finger2_inner_knuckle_joint': 1.49462955,
        'gripper_finger1_finger_joint': 0.4563942,
        'gripper_finger2_finger_joint': 0.4563942,
        'gripper_finger1_finger_tip_joint': 1.49462955,
        'gripper_finger2_finger_tip_joint': 1.49462955,
    }

    def set_gripper_position(position, force=100):
        """Set AG-95 gripper position with mimic joint behavior"""
        for joint_name, joint_id in gripper_joints.items():
            multiplier = gripper_mimic_config.get(joint_name, 1.0)
            target = position * multiplier
            p.setJointMotorControl2(
                robot, joint_id, p.POSITION_CONTROL,
                targetPosition=target, force=force, maxVelocity=2.0
            )

    # Initialize gripper to open position
    if gripper_joints:
        set_gripper_position(0.0)

    # Joint limits
    joint_limits = []
    for j in arm_joints:
        info = p.getJointInfo(robot, j)
        lo, hi = float(info[8]), float(info[9])
        if lo > hi or abs(lo) > 10:
            lo, hi = -2.8, 2.8
        joint_limits.append((lo, hi))

    # Add target block (the object to pick up)
    target_pos = np.array([0.5, -0.25, tabletop_z + 0.025])
    target_id = p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025]*3),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025]*3, rgbaColor=[0.1, 0.2, 0.8, 1]),
        basePosition=target_pos.tolist()
    )

    obstacle_pos = np.array([0.5, 0., tabletop_z + 0.15])
    obstacle = p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.3),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.3, rgbaColor=[0.8, 0.2, 0.1, 1]),
        basePosition=obstacle_pos.tolist()
    )
    obstacles = [obstacle, table]

    # OPTIMAL HOME CONFIGURATION for pick-and-place with center obstacle
    # This configuration:
    # - End effector HIGH but EXTENDED toward workspace/goal area
    # - Centered between pick (y=-0.25) and place (y=+0.25) positions
    # - Clear of table surface and obstacle
    # - Gripper pointing down for immediate pick/place readiness
    home_config = [
        math.radians(0.),      # j1: base rotation - centered between targets
        math.radians(-70.),    # j2: shoulder - elevated but reaching forward
        math.radians(90.),     # j3: elbow - extended toward goal area
        math.radians(-110.),   # j4: wrist1 - compensate to keep gripper pointing down
        math.radians(-90.),    # j5: wrist2 - tool pointing down for pick/place
        math.radians(30.)      # j6: wrist3 - neutral gripper rotation
    ][:len(arm_joints)]
    for j, theta in zip(arm_joints, home_config):
        p.resetJointState(robot, j, theta)

    # RRT-Connect functions
    def get_q():
        return np.array([p.getJointState(robot, j)[0] for j in arm_joints])

    COLLISION_MARGIN = 0.02  # 2cm safety margin

    def in_collision(q):
        """Check if configuration q causes collision with obstacles"""
        orig = [(p.getJointState(robot, j)[0], p.getJointState(robot, j)[1]) for j in arm_joints]
        collision_found = False
        try:
            for j, val in zip(arm_joints, q):
                p.resetJointState(robot, j, float(val))
            p.performCollisionDetection()

            # Check collision with obstacle cylinder - use safety margin
            for pt in p.getClosestPoints(robot, obstacle, distance=COLLISION_MARGIN):
                if pt[8] < COLLISION_MARGIN / 2:
                    collision_found = True
                    break

            # Check collision with table (only deep penetrations)
            if not collision_found:
                for pt in p.getContactPoints(bodyA=robot, bodyB=table):
                    link_index = pt[3]
                    contact_z = pt[5][2]
                    penetration = pt[8]
                    if link_index != -1 and penetration < -0.005 and contact_z < tabletop_z:
                        collision_found = True
                        break

            return collision_found
        finally:
            for j, (pos, vel) in zip(arm_joints, orig):
                p.resetJointState(robot, j, pos, targetVelocity=vel)

    def edge_free(q_a, q_b, n=20):
        for s in np.linspace(0, 1, n):
            if in_collision((1-s)*q_a + s*q_b):
                return False
        return True

    def rrt_connect(q_start, q_goal, step=0.1, max_iter=3000):
        Ta, Tb = [q_start.copy()], [q_goal.copy()]
        Ta_p, Tb_p = [-1], [-1]

        def nearest(T, q):
            return min(range(len(T)), key=lambda i: np.linalg.norm(T[i]-q))

        def steer(q_from, q_to):
            v = q_to - q_from
            d = np.linalg.norm(v)
            return q_from + (v/d)*min(step, d) if d > 1e-12 else q_to.copy()

        def extend(T, Tp, q_target):
            i = nearest(T, q_target)
            q_new = steer(T[i], q_target)
            if edge_free(T[i], q_new, 10):
                T.append(q_new)
                Tp.append(i)
                return len(T)-1
            return None

        def connect(T, Tp, q_target):
            last = None
            while True:
                idx = extend(T, Tp, q_target)
                if idx is None:
                    return last
                last = idx
                if np.linalg.norm(T[idx] - q_target) < 1e-6:
                    return idx

        for _ in range(max_iter):
            q_rand = q_goal.copy() if random.random() < 0.2 else \
                     np.array([random.uniform(lo, hi) for lo, hi in joint_limits])
            a_new = extend(Ta, Ta_p, q_rand)
            if a_new is None:
                Ta, Tb = Tb, Ta
                Ta_p, Tb_p = Tb_p, Ta_p
                continue
            b_last = connect(Tb, Tb_p, Ta[a_new])
            if b_last and np.linalg.norm(Tb[b_last] - Ta[a_new]) < 1e-6:
                # Build path
                path_a, idx = [], a_new
                while idx != -1:
                    path_a.append(Ta[idx])
                    idx = Ta_p[idx]
                path_a.reverse()
                path_b, idx = [], b_last
                while idx != -1:
                    path_b.append(Tb[idx])
                    idx = Tb_p[idx]
                return path_a + path_b[1:][::-1] if len(Tb) == len(Tb_p) else path_a + list(reversed(path_b[1:]))
            Ta, Tb = Tb, Ta
            Ta_p, Tb_p = Tb_p, Ta_p
        return None

    def move_to(q_goal):
        path = rrt_connect(get_q(), np.array(q_goal))
        if path is None:
            print("Planning failed!")
            return False

        print(f"Path found with {len(path)} waypoints")

        # Generate smooth trajectory using minimum-jerk interpolation
        full_trajectory = []
        for i in range(len(path)-1):
            # Use minimum-jerk for each segment
            segment = TrajectoryInterpolator.minimum_jerk(path[i], path[i+1], num_points=30)
            if i == 0:
                full_trajectory.extend(segment)
            else:
                full_trajectory.extend(segment[1:])  # Skip first point to avoid duplicates

        # Execute with smooth control
        for i, q in enumerate(full_trajectory):
            p.setJointMotorControlArray(robot, arm_joints, p.POSITION_CONTROL,
                                        targetPositions=q.tolist(),
                                        targetVelocities=[0.0]*len(arm_joints),
                                        positionGains=[0.12]*len(arm_joints),
                                        velocityGains=[0.35]*len(arm_joints),
                                        forces=[120]*len(arm_joints))
            p.stepSimulation()
            time.sleep(DT)
            if i % 50 == 0:
                print(f"  Progress: {i}/{len(full_trajectory)}")

        print("Move completed")
        return True

    print("\nWaiting 2 seconds before motion...")
    time.sleep(2)

    # Use grasp_frame for IK (proper TCP for gripper)
    ik_link = grasp_frame if grasp_frame is not None else end_effector
    ee_quat = p.getQuaternionFromEuler([math.pi, 0, 0])

    def compute_ik(target_xyz, rest_pose=None):
        """Compute IK for target position with gripper pointing down"""
        if rest_pose is None:
            rest_pose = home_config
        return p.calculateInverseKinematics(
            robot, ik_link, target_xyz, ee_quat,
            lowerLimits=[lo for lo, hi in joint_limits],
            upperLimits=[hi for lo, hi in joint_limits],
            jointRanges=[hi-lo for lo, hi in joint_limits],
            restPoses=rest_pose,
            solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
        )

    def descend_to(target_xyz, steps=50):
        """Descend end-effector smoothly to target position"""
        current_state = p.getLinkState(robot, ik_link)
        start_pos = np.array(current_state[0])
        end_pos = np.array(target_xyz)

        q_current = get_q()
        for i in range(steps):
            alpha = (i + 1) / steps
            # Smooth interpolation using minimum-jerk profile
            s = 10 * alpha**3 - 15 * alpha**4 + 6 * alpha**5
            pos = start_pos + s * (end_pos - start_pos)
            theta = compute_ik(pos.tolist(), rest_pose=list(q_current))
            q_target = np.array(theta[:len(arm_joints)])

            # Execute step
            p.setJointMotorControlArray(robot, arm_joints, p.POSITION_CONTROL,
                                        targetPositions=q_target.tolist(),
                                        positionGains=[0.15]*len(arm_joints),
                                        velocityGains=[0.4]*len(arm_joints),
                                        forces=[150]*len(arm_joints))
            for _ in range(4):
                p.stepSimulation()
                time.sleep(DT)
            q_current = q_target

    def open_gripper(duration=0.8):
        """Open the gripper smoothly"""
        nonlocal grasp_constraint
        print("  Opening gripper...")

        # Release grasp constraint first
        if grasp_constraint is not None:
            p.removeConstraint(grasp_constraint)
            grasp_constraint = None
            print("    Grasp constraint released")

        if gripper_joints:
            steps = int(duration / DT)
            for i in range(steps):
                pos = 0.5 * (1 - i/steps)  # Gradually open from current to 0
                set_gripper_position(pos, force=80)
                p.stepSimulation()
                time.sleep(DT)
            set_gripper_position(0.0, force=80)
        for _ in range(30):
            p.stepSimulation()
            time.sleep(DT)

    grasp_constraint = None  # Will hold constraint ID when grasping

    def close_gripper(duration=1.0):
        """Close the gripper to grasp object"""
        nonlocal grasp_constraint
        print("  Closing gripper...")

        if gripper_joints:
            steps = int(duration / DT)
            object_grasped = False
            for i in range(steps):
                pos = 0.55 * (i / steps)  # Gradually close from 0 to 0.55
                set_gripper_position(pos, force=150)
                p.stepSimulation()
                time.sleep(DT)
                # Check for contact
                contacts = p.getContactPoints(bodyA=robot, bodyB=target_id)
                if len(contacts) > 0:
                    total_force = sum(c[9] for c in contacts)
                    if total_force > 3.0:
                        print(f"    Object contact detected (force={total_force:.1f}N)")
                        object_grasped = True
                        # Close a bit more to secure grip
                        for _ in range(40):
                            set_gripper_position(pos + 0.08, force=200)
                            p.stepSimulation()
                            time.sleep(DT)
                        break

            # Create a constraint to hold the object (backup for physics grip)
            if object_grasped and grasp_constraint is None:
                # Get gripper and object positions
                gripper_state = p.getLinkState(robot, ik_link)
                gripper_pos = gripper_state[0]
                gripper_orn = gripper_state[1]
                obj_pos, obj_orn = p.getBasePositionAndOrientation(target_id)

                # Calculate relative position of object in gripper frame
                inv_gripper_pos, inv_gripper_orn = p.invertTransform(gripper_pos, gripper_orn)
                rel_pos, rel_orn = p.multiplyTransforms(inv_gripper_pos, inv_gripper_orn, obj_pos, obj_orn)

                grasp_constraint = p.createConstraint(
                    parentBodyUniqueId=robot,
                    parentLinkIndex=ik_link,
                    childBodyUniqueId=target_id,
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=rel_pos,
                    childFramePosition=[0, 0, 0],
                    parentFrameOrientation=rel_orn,
                    childFrameOrientation=[0, 0, 0, 1]
                )
                p.changeConstraint(grasp_constraint, maxForce=100)
                print("    Grasp constraint created")

        for _ in range(30):
            p.stepSimulation()
            time.sleep(DT)

    def get_cube_position():
        """Get current cube position"""
        pos, _ = p.getBasePositionAndOrientation(target_id)
        return np.array(pos)

    # ============== PICK AND PLACE SEQUENCE ==============
    initial_cube_pos = get_cube_position()
    print(f"\nInitial cube position: [{initial_cube_pos[0]:.3f}, {initial_cube_pos[1]:.3f}, {initial_cube_pos[2]:.3f}]")

    # 1. Move to above target
    print("\n--- [1/7] Moving above target (blue block) ---")
    above_target = [target_pos[0], target_pos[1], tabletop_z + 0.25]
    theta = compute_ik(above_target)
    move_to(theta[:len(arm_joints)])
    time.sleep(0.5)

    # 2. Open gripper
    print("\n--- [2/7] Opening gripper ---")
    open_gripper()

    # 3. Descend to grasp
    print("\n--- [3/7] Descending to grasp ---")
    grasp_height = target_pos[2] + 0.02  # Slightly above cube center
    grasp_pos = [target_pos[0], target_pos[1], grasp_height]
    descend_to(grasp_pos, steps=60)
    time.sleep(0.3)

    # 4. Close gripper to grasp
    print("\n--- [4/7] Grasping object ---")
    close_gripper()
    time.sleep(0.3)

    # 5. Lift object
    print("\n--- [5/7] Lifting object ---")
    lift_pos = [target_pos[0], target_pos[1], tabletop_z + 0.30]
    descend_to(lift_pos, steps=50)
    lifted_cube_pos = get_cube_position()
    print(f"    Cube position after lift: [{lifted_cube_pos[0]:.3f}, {lifted_cube_pos[1]:.3f}, {lifted_cube_pos[2]:.3f}]")
    time.sleep(0.5)

    # 6. Move to place position (other side of obstacle)
    print("\n--- [6/7] Moving to place position ---")
    place_above = [0.5, 0.25, tabletop_z + 0.30]
    theta = compute_ik(place_above)
    move_to(theta[:len(arm_joints)])
    time.sleep(0.5)

    # 7. Lower and release
    print("\n--- [7/7] Lowering and releasing ---")
    place_pos = [0.5, 0.25, tabletop_z + 0.08]
    descend_to(place_pos, steps=40)
    open_gripper()
    time.sleep(0.5)

    # Return home
    print("\n--- Returning home ---")
    # First lift up
    lift_after_place = [0.5, 0.25, tabletop_z + 0.30]
    descend_to(lift_after_place, steps=30)
    # Then go home
    move_to(home_config)

    # Verify cube was moved
    final_cube_pos = get_cube_position()
    print(f"\nFinal cube position: [{final_cube_pos[0]:.3f}, {final_cube_pos[1]:.3f}, {final_cube_pos[2]:.3f}]")
    displacement = np.linalg.norm(final_cube_pos[:2] - initial_cube_pos[:2])
    if displacement > 0.1:
        print(f"SUCCESS: Cube moved {displacement:.3f}m from pick to place position!")
    else:
        print(f"WARNING: Cube displacement only {displacement:.3f}m - pick-and-place may have failed")

    print("\n" + "=" * 60)
    print("Simulation complete!")
    if GUI:
        print("Close window to exit.")
    print("=" * 60)

    if GUI:
        while True:
            p.stepSimulation()
            time.sleep(DT)
    else:
        p.disconnect()


def main():
    parser = argparse.ArgumentParser(description='FR5 Robot Simulation Example')
    parser.add_argument('--ros2', action='store_true', help='Run as ROS 2 service client')
    parser.add_argument('--headless', action='store_true', help='Run without GUI (for servers/WSL)')
    args = parser.parse_args()

    if args.ros2:
        run_ros2_client()
    else:
        run_standalone(headless=args.headless)


if __name__ == "__main__":
    main()
