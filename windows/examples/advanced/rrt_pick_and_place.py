# INSTRUCTIONS -----------------------------------------------------------------------
#
# INSTALL pybullet before you run this code, by typing the following in the terminal:
# pip install pybullet numpy
#
# RUN this code by typing the following in the terminal:
# python first_example.py
#
# This script demonstrates:
# - RRT-Connect bidirectional path planning for collision-free motion
# - Pick-and-place task with obstacle avoidance
# - Minimum-jerk trajectory interpolation for smooth motion
# - Franka-style parallel gripper for grasping objects
# - Home position similar to Franka Panda robot
# - Smooth gripper motion with contact detection
#
# ------------------------------------------------------------------------------------

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import os
import random
from trajectory_interpolator import TrajectoryInterpolator

# -------- USER SETTINGS --------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Use URDF with fully articulated AG-95 gripper
URDF_PATH = os.path.join(SCRIPT_DIR, "urdf", "fairino5_v6_with_ag95.urdf")
GUI = True  # shows the simulation when True
DT = 1/120.0  # speed of the simulation
COLLISION_MARGIN = 0.02  # 2cm safety margin for collision detection

# -------- UNIFIED CONTROL PARAMETERS --------
# These unified gains prevent jerking from abrupt gain changes
CONTROL_PARAMS = {
    'position_gain': 0.12,      # Balanced - not too aggressive
    'velocity_gain': 0.35,      # Provides smooth damping
    'force': 120,               # Moderate force limit
    'settling_steps': 180,      # 1.5 seconds at 120Hz
    'gain_ramp_steps': 60,      # Smooth gain transition over 0.5s
    'max_joint_velocity': 1.5,  # rad/s - prevents jerky motion
}

# RRT parameters optimized for smooth paths
RRT_PARAMS = {
    'step_size': 0.10,          # Larger steps = fewer waypoints = smoother
    'max_iters': 4000,
    'goal_bias': 0.15,          # Less bias = smoother exploration
    'shortcut_iterations': 400,
    'edge_checks': 20,          # Collision checks per edge
}
# -------------------------------


class RobotStatePkg:
    """Simulates the robot_state_pkg with current joint positions"""
    def __init__(self, robot_sim):
        self._robot_sim = robot_sim

    @property
    def jt_cur_pos(self):
        """Returns current joint positions in degrees"""
        return self._robot_sim._get_current_joint_positions()


class RobotSim:
    """PyBullet simulation wrapper that mimics the Robot.RPC interface with RRT-Connect path planning"""

    @staticmethod
    def _patch_urdf_for_pybullet(urdf_path):
        """Patch URDF file to replace package:// mesh paths with absolute paths"""
        import tempfile
        import re

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

    def __init__(self, urdf_path=None, gui=True):
        """
        Connect to PyBullet simulation (similar to Robot.RPC('IP'))

        Args:
            urdf_path: Path to robot URDF file
            gui: Whether to show GUI (True) or run headless (False)
        """
        # Connect to PyBullet
        self.physics_client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(DT)
        # Camera: distance=2m, yaw=45°, pitch=-30°, target=[0.5, 0, 0.5]
        p.resetDebugVisualizerCamera(2.0, 45, -30, [0.5, 0, 0.5])
        p.setGravity(0, 0, -9.81)

        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")

        # ADD a table to the environment
        self.table_xyz = [0.5, 0, 0]
        self.table = p.loadURDF("table/table.urdf", self.table_xyz)
        self.tabletop_z = p.getAABB(self.table)[1][2]

        # Load robot URDF on the table
        if urdf_path is None:
            urdf_path = URDF_PATH

        # Patch URDF to fix mesh paths for PyBullet
        patched_urdf_path = self._patch_urdf_for_pybullet(urdf_path)

        # Position robot at original position and orientation
        base_xyz = [0.0, 0.0, self.tabletop_z + 0.001]
        # Rotate robot 180 degrees around Z-axis to face the table
        base_orientation = p.getQuaternionFromEuler([0, 0, math.pi])
        self.robot_id = p.loadURDF(
            patched_urdf_path,
            basePosition=base_xyz,
            baseOrientation=base_orientation,
            useFixedBase=True
        )

        # Find revolute joints (controllable joints), finger joints, and grasp frame
        self.revolute_joints = []
        self.finger_joint_indices = []
        self.gripper_joints = {}  # Dict to store AG-95 gripper joints by name
        self.end_effector_index = None
        self.grasp_frame_index = None
        num_joints = p.getNumJoints(self.robot_id)

        print(f"Scanning {num_joints} joints...")
        for j in range(num_joints):
            info = p.getJointInfo(self.robot_id, j)
            joint_type = info[2]
            joint_name = info[1].decode()
            link_name = info[12].decode()

            # Arm joints (j1-j6, revolute, excluding gripper joints)
            if joint_type == p.JOINT_REVOLUTE and "gripper" not in joint_name.lower() and "finger" not in joint_name.lower():
                self.revolute_joints.append(j)
                print(f"  Arm joint {j}: {joint_name} -> {link_name}")

            # AG-95 gripper joints (revolute)
            if "gripper" in joint_name.lower() and joint_type == p.JOINT_REVOLUTE:
                self.gripper_joints[joint_name] = j
                print(f"  Gripper joint {j}: {joint_name}")

            # Legacy finger joints (prismatic) for old URDF
            if "finger" in joint_name.lower() and joint_type == p.JOINT_PRISMATIC:
                self.finger_joint_indices.append(j)
                print(f"  Finger joint {j}: {joint_name} (prismatic)")

            # End effector / grasp frame
            if "wrist3" in link_name.lower():
                self.end_effector_index = j
            if "grasp" in link_name.lower():
                self.grasp_frame_index = j

        # Use last revolute joint if wrist3 not found
        if self.end_effector_index is None and self.revolute_joints:
            self.end_effector_index = self.revolute_joints[-1]

        # Use end effector as grasp frame if not found
        if self.grasp_frame_index is None:
            self.grasp_frame_index = self.end_effector_index

        # Get joint limits
        self.joint_limits = []
        for j in self.revolute_joints:
            info = p.getJointInfo(self.robot_id, j)
            lo, hi = float(info[8]), float(info[9])
            if lo > hi or abs(lo) > 10 or abs(hi) > 10:
                lo, hi = -2.8, 2.8
            self.joint_limits.append((lo, hi))

        # ADD a target block on the table
        self.target_pos = np.array([0.5, -0.25, self.tabletop_z + 0.025])
        self.target_id = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025],
                                                     rgbaColor=[0.1, 0.2, 0.8, 1]),
            basePosition=self.target_pos.tolist()
        )

        # ADD a collision obstacle on the table (STATIC - mass=0)
        self.obstacle_pos = np.array([0.5, 0., self.tabletop_z + 0.15])
        self.obstacle_id = p.createMultiBody(
            baseMass=0,  # Static obstacle - won't move
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.3),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.3,
                                                     rgbaColor=[0.8, 0.2, 0.1, 1]),
            basePosition=self.obstacle_pos.tolist()
        )

        # Obstacle list for collision checking
        self.obstacle_ids = [self.obstacle_id, self.table]

        # Create robot_state_pkg property
        self.robot_state_pkg = RobotStatePkg(self)

        # Grasp constraint for fixed gripper
        self.grasp_constraint = None

        # OPTIMAL HOME CONFIGURATION for pick-and-place with center obstacle
        # This configuration:
        # - End effector HIGH but EXTENDED toward workspace/goal area
        # - Centered between pick (y=-0.25) and place (y=+0.25) positions
        # - Clear of table surface and obstacle
        # - Gripper pointing down for immediate pick/place readiness
        self.home_config = [
            math.radians(0.),      # j1: base rotation - centered between targets
            math.radians(-70.),    # j2: shoulder - elevated but reaching forward
            math.radians(90.),     # j3: elbow - extended toward goal area
            math.radians(-110.),    # j4: wrist1 - compensate to keep gripper pointing down
            math.radians(-90.),    # j5: wrist2 - tool pointing down for pick/place
            math.radians(30.)       # j6: wrist3 - neutral gripper rotation
        ]

        # AG-95 gripper joints setup
        if self.gripper_joints:
            print(f"AG-95 Gripper found with {len(self.gripper_joints)} joints")
            # Initialize gripper to open position
            self._set_gripper_position(0.0)
        elif self.finger_joint_indices:
            print(f"Legacy gripper found with {len(self.finger_joint_indices)} finger joints")
            for joint_id in self.finger_joint_indices:
                p.resetJointState(self.robot_id, joint_id, 0.04)
        else:
            print("No gripper joints found")

        self._reset_to_home()

        print(f"\nRobotSim connected:")
        print(f"  - Arm joints: {len(self.revolute_joints)}")
        print(f"  - Finger joints: {len(self.finger_joint_indices)}")
        print(f"  - End effector index: {self.end_effector_index}")
        print(f"  - Grasp frame index: {self.grasp_frame_index}")
        print(f"Environment: table, target block (blue), obstacle (red cylinder)")

    def _get_finger_position(self):
        """Get current gripper finger position (average of both fingers)"""
        if self.gripper_joints and 'gripper_finger1_joint' in self.gripper_joints:
            return p.getJointState(self.robot_id, self.gripper_joints['gripper_finger1_joint'])[0]
        elif self.finger_joint_indices:
            positions = [p.getJointState(self.robot_id, j)[0] for j in self.finger_joint_indices]
            return np.mean(positions)
        return 0.0

    def _set_gripper_position(self, position, force=100):
        """Set AG-95 gripper position with mimic joint behavior.

        Args:
            position: Main finger joint angle (0 = open, ~0.65 = closed)
            force: Motor force
        """
        if not self.gripper_joints:
            return

        # Mimic multipliers from AG-95 URDF
        mimic_config = {
            'gripper_finger1_joint': 1.0,
            'gripper_finger2_joint': 1.0,
            'gripper_finger1_inner_knuckle_joint': 1.49462955,
            'gripper_finger2_inner_knuckle_joint': 1.49462955,
            'gripper_finger1_finger_joint': 0.4563942,
            'gripper_finger2_finger_joint': 0.4563942,
            'gripper_finger1_finger_tip_joint': 1.49462955,
            'gripper_finger2_finger_tip_joint': 1.49462955,
        }

        for joint_name, joint_id in self.gripper_joints.items():
            multiplier = mimic_config.get(joint_name, 1.0)
            target = position * multiplier
            p.setJointMotorControl2(
                self.robot_id, joint_id, p.POSITION_CONTROL,
                targetPosition=target, force=force, maxVelocity=2.0
            )

    def _reset_to_home(self):
        """Reset robot to home configuration smoothly with gain ramping"""
        print("Resetting to home position...")

        # Get current position for smooth transition
        q_current = self._get_arm_q()

        # Generate smooth minimum-jerk trajectory to home
        num_steps = 240  # 2 seconds at 120Hz
        trajectory = TrajectoryInterpolator.minimum_jerk(q_current, np.array(self.home_config), num_steps)

        # Execute with smooth gain ramping to prevent jerking
        start_gain = 0.03
        target_gain = CONTROL_PARAMS['position_gain']
        ramp_steps = CONTROL_PARAMS['gain_ramp_steps']

        for i, q in enumerate(trajectory):
            # Smooth gain ramp using smoothstep function
            if i < ramp_steps:
                alpha = i / ramp_steps
                alpha = alpha * alpha * (3 - 2 * alpha)  # Smoothstep
                pos_gain = start_gain + alpha * (target_gain - start_gain)
                vel_gain = 0.08 + alpha * (CONTROL_PARAMS['velocity_gain'] - 0.08)
            else:
                pos_gain = target_gain
                vel_gain = CONTROL_PARAMS['velocity_gain']

            p.setJointMotorControlArray(
                self.robot_id, self.revolute_joints, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in q],
                targetVelocities=[0.0]*len(self.revolute_joints),
                positionGains=[pos_gain]*len(self.revolute_joints),
                velocityGains=[vel_gain]*len(self.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(self.revolute_joints),
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        # Open gripper (integrated gripper with prismatic joints)
        if self.finger_joint_indices:
            for joint_id in self.finger_joint_indices:
                p.resetJointState(self.robot_id, joint_id, 0.04, targetVelocity=0.0)
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL,
                                       targetPosition=0.04, force=60, maxVelocity=0.2)

        # Stabilize with unified gains
        for _ in range(CONTROL_PARAMS['settling_steps'] // 3):
            p.setJointMotorControlArray(
                self.robot_id, self.revolute_joints, p.POSITION_CONTROL,
                targetPositions=self.home_config,
                targetVelocities=[0.0]*len(self.revolute_joints),
                positionGains=[CONTROL_PARAMS['position_gain']]*len(self.revolute_joints),
                velocityGains=[CONTROL_PARAMS['velocity_gain']]*len(self.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(self.revolute_joints),
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        print("Home position reached")

    def open_gripper(self, duration=1.0):
        """Open the gripper"""
        print("Opening gripper...")

        # Release any grasp constraint
        if hasattr(self, 'grasp_constraint') and self.grasp_constraint is not None:
            p.removeConstraint(self.grasp_constraint)
            self.grasp_constraint = None
            print("  Released grasp constraint")

        # AG-95 gripper (revolute joints)
        if self.gripper_joints:
            current_pos = self._get_finger_position()
            target_pos = 0.0  # Open position
            num_steps = int(duration / DT)
            trajectory = TrajectoryInterpolator.minimum_jerk(
                np.array([current_pos]),
                np.array([target_pos]),
                num_steps
            )
            for pos_array in trajectory:
                self._set_gripper_position(float(pos_array[0]))
                p.stepSimulation()
                time.sleep(DT if GUI else 0)
            print(f"  AG-95 gripper opened")
        # Legacy prismatic finger joints
        elif self.finger_joint_indices:
            current_pos = self._get_finger_position()
            target_pos = 0.04
            num_steps = int(duration / DT)
            trajectory = TrajectoryInterpolator.minimum_jerk(
                np.array([current_pos]),
                np.array([target_pos]),
                num_steps
            )
            for pos_array in trajectory:
                pos = float(pos_array[0])
                for joint_id in self.finger_joint_indices:
                    p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL,
                                           targetPosition=pos, force=40, maxVelocity=0.15)
                p.stepSimulation()
                time.sleep(DT if GUI else 0)
            print(f"  Gripper opened to {target_pos*2000:.0f}mm")
        else:
            for _ in range(int(duration / DT)):
                p.stepSimulation()
                time.sleep(DT if GUI else 0)
            print("  No gripper to open")

    def close_gripper(self, duration=1.0):
        """Close the gripper to grasp object"""
        print("Closing gripper...")

        # AG-95 gripper (revolute joints)
        if self.gripper_joints:
            current_pos = self._get_finger_position()
            target_pos = 0.5  # Close position (max ~0.65)
            num_steps = int(duration / DT)
            trajectory = TrajectoryInterpolator.minimum_jerk(
                np.array([current_pos]),
                np.array([target_pos]),
                num_steps
            )
            object_grasped = False
            for pos_array in trajectory:
                self._set_gripper_position(float(pos_array[0]), force=150)
                p.stepSimulation()
                time.sleep(DT if GUI else 0)
                # Check for contact with target object
                if hasattr(self, 'target_id'):
                    contacts = p.getContactPoints(bodyA=self.robot_id, bodyB=self.target_id)
                    if len(contacts) > 0:
                        total_force = sum(c[9] for c in contacts)
                        if total_force > 5.0:
                            object_grasped = True
                            # Keep closing a bit more to secure grip
                            for _ in range(30):
                                self._set_gripper_position(float(pos_array[0]) + 0.05, force=150)
                                p.stepSimulation()
                                time.sleep(DT if GUI else 0)
                            break
            print(f"  AG-95 gripper closed" + (" - object grasped!" if object_grasped else ""))
        # Legacy prismatic finger joints
        elif self.finger_joint_indices:
            current_pos = self._get_finger_position()
            target_pos = 0.005
            num_steps = int(duration / DT)
            trajectory = TrajectoryInterpolator.minimum_jerk(
                np.array([current_pos]),
                np.array([target_pos]),
                num_steps
            )
            object_grasped = False
            for pos_array in trajectory:
                pos = float(pos_array[0])
                for joint_id in self.finger_joint_indices:
                    p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL,
                                           targetPosition=pos, force=60, maxVelocity=0.1)
                p.stepSimulation()
                time.sleep(DT if GUI else 0)
                if hasattr(self, 'target_id'):
                    contacts = p.getContactPoints(bodyA=self.robot_id, bodyB=self.target_id)
                    if len(contacts) > 0 and sum(c[9] for c in contacts) > 5.0:
                        object_grasped = True
                        break
            final_pos = self._get_finger_position()
            print(f"  Gripper closed to {final_pos*2000:.0f}mm" + (" - object grasped!" if object_grasped else ""))
        else:
            # Use constraint-based grasping for fixed gripper
            for _ in range(int(duration / DT)):
                p.stepSimulation()
                time.sleep(DT if GUI else 0)

            if hasattr(self, 'target_id') and hasattr(self, 'grasp_frame_index') and self.grasp_frame_index:
                grasp_state = p.getLinkState(self.robot_id, self.grasp_frame_index)
                grasp_pos = grasp_state[0]
                target_pos, _ = p.getBasePositionAndOrientation(self.target_id)
                dist = np.linalg.norm(np.array(grasp_pos) - np.array(target_pos))

                if dist < 0.1:
                    self.grasp_constraint = p.createConstraint(
                        self.robot_id, self.grasp_frame_index,
                        self.target_id, -1,
                        p.JOINT_FIXED,
                        [0, 0, 0], [0, 0, 0], [0, 0, 0]
                    )
                    print(f"  Object grasped (constraint, dist={dist:.3f}m)")
                else:
                    print(f"  Object too far (dist={dist:.3f}m)")
                    self.grasp_constraint = None
            else:
                self.grasp_constraint = None
                print("  No gripper to close")

    def _get_current_joint_positions(self):
        """Get current joint positions in degrees"""
        positions = []
        for j in self.revolute_joints:
            pos = p.getJointState(self.robot_id, j)[0]
            positions.append(math.degrees(pos))
        return positions

    def _get_arm_q(self):
        """Get current arm joint angles in radians"""
        return np.array([p.getJointState(self.robot_id, j)[0] for j in self.revolute_joints], dtype=float)

    # ========================= RRT-CONNECT PATH PLANNING =========================

    def _in_collision(self, q, self_collisions=False, margin=None):
        """
        Check if configuration q causes collision with obstacles.

        Args:
            q: Joint configuration to check
            self_collisions: Whether to check self-collisions
            margin: Collision margin in meters (uses COLLISION_MARGIN if None)

        Returns:
            True if collision detected, False otherwise
        """
        if margin is None:
            margin = COLLISION_MARGIN

        # Save original joint states
        original_states = [(p.getJointState(self.robot_id, j)[0], p.getJointState(self.robot_id, j)[1])
                           for j in self.revolute_joints]
        collision_found = False

        try:
            # Set robot to test configuration
            for j, val in zip(self.revolute_joints, q):
                p.resetJointState(self.robot_id, j, float(val))

            # Step simulation to update collision geometry
            p.performCollisionDetection()

            # Check self-collisions if enabled
            if self_collisions:
                pts = p.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id)
                if len(pts) > 0:
                    collision_found = True

            # Check collisions with obstacle cylinder - use safety margin
            if not collision_found:
                pts = p.getClosestPoints(bodyA=self.robot_id, bodyB=self.obstacle_id, distance=margin)
                for pt in pts:
                    # pt[8] is the distance (negative = penetration, positive = gap)
                    # Collision if penetrating OR within safety margin
                    if pt[8] < margin / 2:  # Use half margin for obstacle proximity
                        collision_found = True
                        break

            # Check collision with table surface
            # Robot base and lower arm links may touch table structure (mounted robot)
            # Only flag as collision if arm/gripper significantly penetrates tabletop
            if not collision_found:
                pts = p.getContactPoints(bodyA=self.robot_id, bodyB=self.table)
                for pt in pts:
                    link_index = pt[3]  # Link index on robot (-1 = base)
                    contact_point = pt[5]  # Position on robot
                    penetration = pt[8]   # Penetration depth (negative = penetrating)
                    # Skip base link contacts (robot is mounted on table)
                    if link_index == -1:
                        continue
                    # Only flag deep penetrations (> 5mm into table surface)
                    if penetration < -0.005 and contact_point[2] < self.tabletop_z:
                        collision_found = True
                        break

            # Check ground plane collision
            if not collision_found:
                pts = p.getContactPoints(bodyA=self.robot_id, bodyB=self.plane_id)
                if len(pts) > 0:
                    collision_found = True

        finally:
            # Restore original joint states
            for j, (pos, vel) in zip(self.revolute_joints, original_states):
                p.resetJointState(self.robot_id, j, pos, targetVelocity=vel)

        return collision_found

    def _edge_collision_free(self, q_a, q_b, num_checks=None):
        """
        Check if path between q_a and q_b is collision-free.

        Uses interpolated checks along the path with safety margin.
        """
        if num_checks is None:
            num_checks = RRT_PARAMS['edge_checks']
        for s in np.linspace(0.0, 1.0, num_checks):
            q = (1 - s) * np.array(q_a) + s * np.array(q_b)
            if self._in_collision(q, self_collisions=False, margin=COLLISION_MARGIN):
                return False
        return True

    def _sample_q(self):
        """Sample random configuration within joint limits."""
        return np.array([random.uniform(lo, hi) for (lo, hi) in self.joint_limits], dtype=float)

    def _steer(self, q_from, q_to, step_size):
        """Steer from q_from toward q_to with max distance step_size."""
        v = q_to - q_from
        d = float(np.linalg.norm(v))
        if d < 1e-12:
            return q_to.copy()
        return q_from + (v / d) * min(step_size, d)

    def _nearest(self, tree_q, q):
        """Find nearest node in tree to q."""
        dmin = float("inf")
        imin = 0
        for i, tq in enumerate(tree_q):
            d = float(np.linalg.norm(q - tq))
            if d < dmin:
                dmin = d
                imin = i
        return imin

    def rrt_connect(self, q_start, q_goal, step_size=None, max_iters=None, goal_bias=None):
        """RRT-Connect bidirectional planning with optimized parameters."""
        # Use RRT_PARAMS defaults if not specified
        if step_size is None:
            step_size = RRT_PARAMS['step_size']
        if max_iters is None:
            max_iters = RRT_PARAMS['max_iters']
        if goal_bias is None:
            goal_bias = RRT_PARAMS['goal_bias']

        Ta_q, Ta_parent = [q_start.copy()], [-1]
        Tb_q, Tb_parent = [q_goal.copy()], [-1]

        def extend(Tq, Tparent, q_target):
            i_near = self._nearest(Tq, q_target)
            q_near = Tq[i_near]
            q_new = self._steer(q_near, q_target, step_size)
            if not self._edge_collision_free(q_near, q_new):
                return None
            Tq.append(q_new)
            Tparent.append(i_near)
            return len(Tq) - 1

        def connect(Tq, Tparent, q_target):
            last = None
            while True:
                idx = extend(Tq, Tparent, q_target)
                if idx is None:
                    return last
                last = idx
                if float(np.linalg.norm(Tq[idx] - q_target)) < 1e-6:
                    return idx

        def path_from_tree(Tq, Tparent, idx):
            out = []
            while idx != -1:
                out.append(Tq[idx])
                idx = Tparent[idx]
            out.reverse()
            return out

        for iteration in range(max_iters):
            q_rand = q_goal.copy() if (random.random() < goal_bias) else self._sample_q()

            a_new = extend(Ta_q, Ta_parent, q_rand)
            if a_new is None:
                Ta_q, Tb_q = Tb_q, Ta_q
                Ta_parent, Tb_parent = Tb_parent, Ta_parent
                continue

            q_new = Ta_q[a_new]
            b_last = connect(Tb_q, Tb_parent, q_new)

            if b_last is not None and float(np.linalg.norm(Tb_q[b_last] - q_new)) < 1e-6:
                path_a = path_from_tree(Ta_q, Ta_parent, a_new)
                path_b = path_from_tree(Tb_q, Tb_parent, b_last)
                path_b.reverse()
                print(f"RRT-Connect succeeded at iteration {iteration}")
                return path_a + path_b[1:]

            Ta_q, Tb_q = Tb_q, Ta_q
            Ta_parent, Tb_parent = Tb_parent, Ta_parent

        print("RRT-Connect failed: max iterations reached")
        return None

    def shortcut_path(self, path, tries=None):
        """Smooth path by removing unnecessary waypoints."""
        if tries is None:
            tries = RRT_PARAMS['shortcut_iterations']
        if path is None or len(path) < 3:
            return path
        path = [q.copy() for q in path]
        shortcuts_made = 0

        for _ in range(tries):
            if len(path) < 3:
                break
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if j - i < 2:
                continue
            if self._edge_collision_free(path[i], path[j]):
                path = path[:i+1] + path[j:]
                shortcuts_made += 1

        print(f"Shortcutting removed {shortcuts_made} waypoints")
        return path

    def _exec_to_q(self, q_target, duration=0.5):
        """
        Smoothly move arm to target configuration using minimum-jerk trajectory.

        Uses unified control parameters to prevent jerking from gain mismatches.

        Args:
            q_target: Target joint configuration (radians)
            duration: Motion duration in seconds (default: 0.5s)
        """
        q_current = self._get_arm_q()
        num_steps = max(int(duration / DT), 10)

        # Check if motion is too fast - clamp velocity
        max_delta = float(np.max(np.abs(np.array(q_target) - q_current)))
        min_duration = max_delta / CONTROL_PARAMS['max_joint_velocity']
        if duration < min_duration:
            duration = min_duration
            num_steps = max(int(duration / DT), 10)

        # Generate minimum-jerk trajectory for smooth motion
        trajectory = TrajectoryInterpolator.minimum_jerk(q_current, np.array(q_target), num_steps)

        # Execute trajectory with unified PID gains
        for q in trajectory:
            p.setJointMotorControlArray(
                self.robot_id, self.revolute_joints, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in q],
                targetVelocities=[0.0]*len(self.revolute_joints),
                positionGains=[CONTROL_PARAMS['position_gain']]*len(self.revolute_joints),
                velocityGains=[CONTROL_PARAMS['velocity_gain']]*len(self.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(self.revolute_joints)
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

    # ========================= END RRT-CONNECT =========================

    def _smooth_rrt_path(self, waypoints, window_size=5):
        """Apply moving average smoothing to RRT waypoints to reduce jerking."""
        if len(waypoints) <= window_size:
            return waypoints

        smoothed = [waypoints[0]]  # Keep start exactly
        for i in range(1, len(waypoints) - 1):
            start = max(0, i - window_size // 2)
            end = min(len(waypoints), i + window_size // 2 + 1)
            window = waypoints[start:end]
            avg = np.mean(window, axis=0)
            smoothed.append(avg)
        smoothed.append(waypoints[-1])  # Keep end exactly
        return smoothed

    def _velocity_continuous_trajectory(self, waypoints, total_time=None):
        """
        Generate velocity-continuous minimum-jerk trajectory through waypoints.

        This ensures C1 continuity (position + velocity) at waypoint boundaries,
        eliminating the jerking caused by zero-velocity stops at each waypoint.
        """
        n_waypoints = len(waypoints)
        if n_waypoints < 2:
            return np.array(waypoints)

        # Calculate segment lengths for proportional time allocation
        segment_lengths = []
        for i in range(n_waypoints - 1):
            dist = float(np.linalg.norm(np.array(waypoints[i+1]) - np.array(waypoints[i])))
            segment_lengths.append(max(dist, 0.01))  # Avoid zero-length segments

        total_length = sum(segment_lengths)

        # Allocate time proportionally, respecting max velocity
        if total_time is None:
            total_time = total_length / (CONTROL_PARAMS['max_joint_velocity'] * 0.7)  # 70% of max
        segment_times = [total_time * (l / total_length) for l in segment_lengths]

        # Compute velocities at waypoints for C1 continuity
        waypoint_velocities = [np.zeros(len(waypoints[0]))]  # Start at rest

        for i in range(1, n_waypoints - 1):
            # Use damped average of incoming/outgoing directions
            v_in = (np.array(waypoints[i]) - np.array(waypoints[i-1])) / segment_times[i-1]
            v_out = (np.array(waypoints[i+1]) - np.array(waypoints[i])) / segment_times[i]
            # Damped average to prevent overshooting
            waypoint_velocities.append((v_in + v_out) * 0.25)

        waypoint_velocities.append(np.zeros(len(waypoints[0])))  # End at rest

        # Generate trajectory with quintic Hermite interpolation for each segment
        trajectory = []
        for seg in range(n_waypoints - 1):
            q0 = np.array(waypoints[seg])
            q1 = np.array(waypoints[seg + 1])
            v0 = waypoint_velocities[seg]
            v1 = waypoint_velocities[seg + 1]
            T = segment_times[seg]

            n_steps = max(int(T / DT), 5)
            for step in range(n_steps):
                tau = step / max(n_steps - 1, 1)

                # Quintic Hermite basis functions for position/velocity boundary conditions
                h00 = 1 - 10*tau**3 + 15*tau**4 - 6*tau**5
                h10 = tau - 6*tau**3 + 8*tau**4 - 3*tau**5
                h01 = 10*tau**3 - 15*tau**4 + 6*tau**5
                h11 = -4*tau**3 + 7*tau**4 - 3*tau**5

                q = h00*q0 + h10*T*v0 + h01*q1 + h11*T*v1
                trajectory.append(q)

        # Ensure final waypoint is included
        trajectory.append(np.array(waypoints[-1]))
        return np.array(trajectory)

    def MoveJ(self, joint_pos, tool=0, user=0, vel=100, acc=100, ovl=100):
        """
        Move robot to joint position using RRT-Connect path planning.

        Features velocity-continuous trajectory generation to prevent jerking.

        Args:
            joint_pos: Target joint positions in degrees
            tool: Tool frame (not used in simulation)
            user: User frame (not used in simulation)
            vel: Velocity (not used in simulation)
            acc: Acceleration (not used in simulation)
            ovl: Override (not used in simulation)

        Returns:
            0 for success, -1 for failure
        """
        _, _, _, _, _ = tool, user, vel, acc, ovl

        if len(joint_pos) != len(self.revolute_joints):
            print(f"Error: Expected {len(self.revolute_joints)} joint positions, got {len(joint_pos)}")
            return -1

        # Convert degrees to radians
        q_goal = np.array([math.radians(pos) for pos in joint_pos], dtype=float)
        q_start = self._get_arm_q()

        # Validate configurations
        if self._in_collision(q_start):
            print("Warning: Start configuration is in collision!")
        if self._in_collision(q_goal):
            print("Warning: Goal configuration is in collision!")
            return -1

        # Plan with RRT-Connect using optimized parameters
        print("Starting RRT-Connect planning...")
        path = self.rrt_connect(q_start, q_goal)

        if path is None:
            print("RRT-Connect failed. Trying direct motion with minimum-jerk trajectory...")
            # Fallback to direct motion using minimum-jerk trajectory
            q_current = self._get_arm_q()
            q_goal_rad = np.array([math.radians(pos) for pos in joint_pos])

            # Generate smooth minimum-jerk trajectory for direct motion
            num_steps = 480  # 4 seconds at 120Hz
            trajectory = TrajectoryInterpolator.minimum_jerk(q_current, q_goal_rad, num_steps)

            # Execute with unified gains and smooth gain ramping
            ramp_steps = CONTROL_PARAMS['gain_ramp_steps']
            start_gain = 0.05
            for i, q in enumerate(trajectory):
                # Smooth gain ramp at start
                if i < ramp_steps:
                    alpha = i / ramp_steps
                    alpha = alpha * alpha * (3 - 2 * alpha)
                    pos_gain = start_gain + alpha * (CONTROL_PARAMS['position_gain'] - start_gain)
                    vel_gain = 0.1 + alpha * (CONTROL_PARAMS['velocity_gain'] - 0.1)
                else:
                    pos_gain = CONTROL_PARAMS['position_gain']
                    vel_gain = CONTROL_PARAMS['velocity_gain']

                p.setJointMotorControlArray(
                    bodyIndex=self.robot_id,
                    jointIndices=self.revolute_joints,
                    controlMode=p.POSITION_CONTROL,
                    targetPositions=[float(x) for x in q],
                    targetVelocities=[0.0] * len(self.revolute_joints),
                    positionGains=[pos_gain] * len(self.revolute_joints),
                    velocityGains=[vel_gain] * len(self.revolute_joints),
                    forces=[CONTROL_PARAMS['force']] * len(self.revolute_joints)
                )
                p.stepSimulation()
                time.sleep(DT if GUI else 0)
            return 0

        print(f"Initial path length: {len(path)} waypoints")
        path = self.shortcut_path(path, tries=RRT_PARAMS['shortcut_iterations'])
        print(f"After shortcutting: {len(path)} waypoints")

        # Apply moving average smoothing to reduce jerky waypoints
        path = self._smooth_rrt_path(path, window_size=5)
        print(f"After smoothing: {len(path)} waypoints")

        # Ensure minimum waypoints for smooth interpolation
        if len(path) < 5:
            print(f"Path too short ({len(path)} points), adding intermediate waypoints...")
            interpolated_path = [path[0]]
            for i in range(len(path) - 1):
                for alpha in np.linspace(0.25, 0.75, 3):
                    interpolated_path.append((1 - alpha) * path[i] + alpha * path[i + 1])
                interpolated_path.append(path[i + 1])
            path = interpolated_path
            print(f"Expanded to {len(path)} waypoints")

        # Generate VELOCITY-CONTINUOUS trajectory (eliminates zero-velocity jerking)
        print("Generating velocity-continuous trajectory...")
        trajectory = self._velocity_continuous_trajectory(path)
        print(f"Final trajectory: {len(trajectory)} points")

        # Settle at current position before executing with smooth gain ramp
        print("Settling before execution...")
        current_arm_q = self._get_arm_q()
        for i in range(CONTROL_PARAMS['settling_steps'] // 3):
            # Gradually ramp up gains during settling
            alpha = min(1.0, i / 30)
            pos_gain = 0.05 + alpha * (CONTROL_PARAMS['position_gain'] - 0.05)
            vel_gain = 0.1 + alpha * (CONTROL_PARAMS['velocity_gain'] - 0.1)

            p.setJointMotorControlArray(
                self.robot_id, self.revolute_joints, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in current_arm_q],
                targetVelocities=[0.0]*len(self.revolute_joints),
                positionGains=[pos_gain]*len(self.revolute_joints),
                velocityGains=[vel_gain]*len(self.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(self.revolute_joints)
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        # Execute trajectory with unified gains
        print("Executing planned path...")
        for i, q in enumerate(trajectory):
            p.setJointMotorControlArray(
                self.robot_id, self.revolute_joints, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in q],
                targetVelocities=[0.0]*len(self.revolute_joints),
                positionGains=[CONTROL_PARAMS['position_gain']]*len(self.revolute_joints),
                velocityGains=[CONTROL_PARAMS['velocity_gain']]*len(self.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(self.revolute_joints)
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)
            if i % 100 == 0:
                print(f"  Progress: {i}/{len(trajectory)}")

        print("Move completed successfully")
        return 0

    def MoveToTarget(self):
        """Move end-effector above target block using IK and RRT-Connect"""
        z_above = self.tabletop_z + 0.35
        target_above = np.array([self.target_pos[0], self.target_pos[1], z_above])
        ee_euler = (math.pi, 0.0, 0.0)
        ee_quat = p.getQuaternionFromEuler(ee_euler)

        # Use joint limits and rest pose for stable IK solutions
        lower_limits = [l for l, _ in self.joint_limits]
        upper_limits = [h for _, h in self.joint_limits]
        joint_ranges = [h - l for l, h in self.joint_limits]

        theta = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, target_above.tolist(), ee_quat,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=self.home_config,  # Bias toward home configuration
            solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
        )
        q_goal = np.array(theta[:len(self.revolute_joints)], dtype=float)
        target_degrees = [math.degrees(q) for q in q_goal]
        return self.MoveJ(target_degrees)

    def MoveToPlace(self):
        """Move end-effector above place position (other side of obstacle)"""
        z_above = self.tabletop_z + 0.35
        place_pos = np.array([0.5, 0.25, z_above])
        ee_euler = (math.pi, 0.0, 0.0)
        ee_quat = p.getQuaternionFromEuler(ee_euler)

        # Use joint limits and rest pose for stable IK solutions
        lower_limits = [l for l, _ in self.joint_limits]
        upper_limits = [h for _, h in self.joint_limits]
        joint_ranges = [h - l for l, h in self.joint_limits]

        theta = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, place_pos.tolist(), ee_quat,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=self.home_config,  # Bias toward home configuration
            solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
        )
        q_goal = np.array(theta[:len(self.revolute_joints)], dtype=float)
        target_degrees = [math.degrees(q) for q in q_goal]
        return self.MoveJ(target_degrees)

    def DescendToGrasp(self, target_z):
        """Descend end-effector to grasp height with collision checking"""
        z_start = self.tabletop_z + 0.35
        z_end = target_z
        ee_euler = (math.pi, 0.0, 0.0)
        ee_quat = p.getQuaternionFromEuler(ee_euler)

        # Use joint limits and current config as rest pose for IK stability
        lower_limits = [l for l, _ in self.joint_limits]
        upper_limits = [h for _, h in self.joint_limits]
        joint_ranges = [h - l for l, h in self.joint_limits]

        print("Descending to grasp pose...")
        num_approach_steps = 70
        q_current = self._get_arm_q()
        for i, z in enumerate(np.linspace(z_start, z_end, num_approach_steps)):
            wp = np.array([self.target_pos[0], self.target_pos[1], float(z)])
            theta = p.calculateInverseKinematics(
                self.robot_id, self.grasp_frame_index, wp.tolist(), ee_quat,
                lowerLimits=lower_limits,
                upperLimits=upper_limits,
                jointRanges=joint_ranges,
                restPoses=list(q_current),  # Use current config to avoid jumps
                solver=p.IK_DLS, maxNumIterations=400, residualThreshold=1e-4
            )
            q_tgt = np.array(theta[:len(self.revolute_joints)], dtype=float)
            steps_duration = 0.1 if i < num_approach_steps - 1 else 0.5
            self._exec_to_q(q_tgt, duration=steps_duration)
            q_current = q_tgt  # Update rest pose for next iteration

        # Settle at grasp pose with unified gains
        q_final = self._get_arm_q()
        for _ in range(100):
            p.setJointMotorControlArray(
                self.robot_id, self.revolute_joints, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in q_final],
                targetVelocities=[0.0]*len(self.revolute_joints),
                positionGains=[CONTROL_PARAMS['position_gain']]*len(self.revolute_joints),
                velocityGains=[CONTROL_PARAMS['velocity_gain']]*len(self.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(self.revolute_joints)
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

    def LiftObject(self, lift_height=0.25):
        """Lift the object after grasping using smooth trajectory"""
        print("Lifting object...")
        lifted_pos = (self.target_pos + np.array([0, 0, lift_height])).tolist()
        ee_euler = (math.pi, 0.0, 0.0)
        ee_quat = p.getQuaternionFromEuler(ee_euler)

        # Use current config as rest pose for smooth IK
        q_current = self._get_arm_q()
        lower_limits = [l for l, _ in self.joint_limits]
        upper_limits = [h for _, h in self.joint_limits]
        joint_ranges = [h - l for l, h in self.joint_limits]

        theta = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, lifted_pos, ee_quat,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=list(q_current),
            solver=p.IK_DLS, maxNumIterations=200, residualThreshold=1e-4
        )
        q_target = np.array(theta[:len(self.revolute_joints)], dtype=float)

        # Use smooth trajectory for lifting
        self._exec_to_q(q_target, duration=1.0)

        # Maintain gripper grip during lift
        if self.finger_joint_indices:
            for joint_id in self.finger_joint_indices:
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL,
                                       targetPosition=self._get_finger_position(),
                                       force=80)  # Higher force to hold object

        # Stabilize with unified gains
        q_final = self._get_arm_q()
        for _ in range(50):
            p.setJointMotorControlArray(
                self.robot_id, self.revolute_joints, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in q_final],
                targetVelocities=[0.0]*len(self.revolute_joints),
                positionGains=[CONTROL_PARAMS['position_gain']]*len(self.revolute_joints),
                velocityGains=[CONTROL_PARAMS['velocity_gain']]*len(self.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(self.revolute_joints)
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

    def ReturnHome(self):
        """Return to home configuration"""
        home_degrees = [math.degrees(q) for q in self.home_config]
        return self.MoveJ(home_degrees)

    def CloseRPC(self):
        """Close the simulation connection (similar to Robot.RPC.CloseRPC)"""
        p.disconnect()
        print("Simulation connection closed.")


def main():
    print("=" * 60)
    print("FR5 Robot Simulation with RRT-Connect Path Planning")
    print("=" * 60)
    print("\nThis simulation demonstrates:")
    print("  - PyBullet environment with table, obstacle, and target")
    print("  - RRT-Connect collision-free path planning")
    print("  - Gripper control (open/close)")
    print("  - Pick-and-place motion with gripper")
    print("=" * 60)

    # Connect to robot simulation
    robot = RobotSim(gui=GUI)

    # Wait for user to see initial setup
    print("\nWaiting 3 seconds before starting motion...")
    time.sleep(3)

    try:
        # Read current joint angles
        currPos = robot.robot_state_pkg.jt_cur_pos
        print(f"\nCurrent joint positions (degrees): {[f'{p:.1f}' for p in currPos]}")

        # CRITICAL: Ensure robot is settled at start configuration before planning
        print("\nSettling robot at start configuration...")
        q_start = robot._get_arm_q()
        for i in range(CONTROL_PARAMS['settling_steps']):
            # Use smooth gain ramp during settling
            alpha = min(1.0, i / 60)
            pos_gain = 0.05 + alpha * (CONTROL_PARAMS['position_gain'] - 0.05)
            vel_gain = 0.1 + alpha * (CONTROL_PARAMS['velocity_gain'] - 0.1)

            p.setJointMotorControlArray(
                robot.robot_id, robot.revolute_joints, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in q_start],
                targetVelocities=[0.0]*len(robot.revolute_joints),
                positionGains=[pos_gain]*len(robot.revolute_joints),
                velocityGains=[vel_gain]*len(robot.revolute_joints),
                forces=[CONTROL_PARAMS['force']]*len(robot.revolute_joints)
            )
            p.stepSimulation()
            time.sleep(DT if GUI else 0)
        print("Robot settled at start configuration")

        # Move to above target (demonstrating RRT path planning around obstacle)
        print("\n--- [1/6] Moving to target position (above blue block) ---")
        rtn = robot.MoveToTarget()
        if rtn == 0:
            print("Reached target position")
        else:
            raise RuntimeError("Failed to reach target position")
        time.sleep(1)

        # Descend to grasp
        print("\n--- [2/6] Descending to grasp ---")
        robot.DescendToGrasp(robot.target_pos[2] + 0.005)
        print("Reached grasp pose")

        # Close gripper to grasp object
        print("\n--- [3/6] Grasping object ---")
        robot.close_gripper()

        # Lift object
        print("\n--- [4/6] Lifting object ---")
        robot.LiftObject()

        # Move to place position (other side of obstacle)
        print("\n--- [5/6] Moving to place position (other side of obstacle) ---")
        rtn = robot.MoveToPlace()
        if rtn == 0:
            print("Reached place position")
        else:
            raise RuntimeError("Failed to reach place position")
        time.sleep(1)

        # Release object
        print("\n--- [6/6] Releasing object ---")
        robot.open_gripper()

        # Return home
        print("\n--- Returning to home position ---")
        rtn = robot.ReturnHome()
        if rtn == 0:
            print("Returned to home")
        else:
            raise RuntimeError("Failed to return home")

        print("\n" + "=" * 60)
        print("✓ Successfully completed pick-and-place task!")
        print("=" * 60)

    except Exception as e:
        import traceback
        print("\n=== ERROR CAUGHT ===")
        print(f"Error: {e}")
        traceback.print_exc()
        print("===================\n")

        if GUI:
            print("Simulation paused. Close window to exit.")
            try:
                while True:
                    p.stepSimulation()
                    time.sleep(DT)
            except Exception:
                # Window was closed - graceful exit
                pass

    print("\nDone! Simulation running... Close window to exit.")
    # Keep simulation running
    if GUI:
        try:
            while True:
                p.stepSimulation()
                time.sleep(DT)
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        except Exception:
            # Window was closed - graceful exit
            pass


if __name__ == "__main__":
    main()
