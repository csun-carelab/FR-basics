# INSTRUCTIONS -----------------------------------------------------------------------
# INSTALL pybullet before you run this code, by typing the following in the terminal:
# pip install pybullet numpy
#
# RUN this code by typing the following in the terminal:
# python pick_and_place_example.py
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
URDF_PATH = os.path.join(SCRIPT_DIR, "urdf", "fairino5_v6.urdf")  # Use Fairino5 v6 URDF
GUI = True  # shows the simulation when True
DT = 1/120.0  # speed of the simulation
# -------------------------------


class FR5PickAndPlace:
    """Complete pick-and-place simulation for FR5 robot with gripper"""

    @staticmethod
    def _patch_urdf_for_pybullet(urdf_path):
        """Patch URDF file to replace relative mesh paths with absolute paths"""
        import tempfile
        import re

        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        meshes_dir = os.path.join(urdf_dir, 'fairino5_v6')

        with open(urdf_path, 'r') as f:
            urdf_content = f.read()

        # Replace package:// paths with absolute paths for fairino description
        urdf_content = re.sub(
            r'package://fairino_description/meshes/fairino5_v6',
            meshes_dir,
            urdf_content
        )
        # Also handle fr5_description package paths (legacy)
        urdf_content = re.sub(
            r'package://fr5_description/meshes',
            meshes_dir,
            urdf_content
        )
        # Replace relative meshes/ paths with absolute paths
        urdf_content = re.sub(
            r'filename="meshes/',
            f'filename="{meshes_dir}/',
            urdf_content
        )

        # Create temporary patched URDF file
        temp_fd, temp_path = tempfile.mkstemp(suffix='.urdf', text=True)
        with os.fdopen(temp_fd, 'w') as f:
            f.write(urdf_content)

        return temp_path

    def __init__(self, gui=True):
        # Connect to PyBullet
        self.physics_client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(DT)
        p.resetDebugVisualizerCamera(2.0, 45, -30, [0.5, 0, 0.5])
        p.setGravity(0, 0, -9.81)

        # Load environment
        self.plane_id = p.loadURDF("plane.urdf")
        self.table_xyz = [0.5, 0, 0]
        self.table = p.loadURDF("table/table.urdf", self.table_xyz)
        self.tabletop_z = p.getAABB(self.table)[1][2]

        # Patch URDF to work with PyBullet
        patched_urdf = self._patch_urdf_for_pybullet(URDF_PATH)

        # Load robot with gripper
        base_xyz = [0.0, 0.0, self.tabletop_z + 0.001]
        base_orientation = p.getQuaternionFromEuler([0, 0, math.pi])
        self.robot_id = p.loadURDF(
            patched_urdf,
            basePosition=base_xyz,
            baseOrientation=base_orientation,
            useFixedBase=True
        )

        # Find arm joints and gripper joints
        self.arm_joint_indices = []
        self.finger_joint_indices = []
        self.grasp_frame_index = None

        for j in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, j)
            joint_name = joint_info[1].decode()
            link_name = joint_info[12].decode()
            joint_type = joint_info[2]

            # Arm joints (first 6 revolute joints)
            if joint_type == p.JOINT_REVOLUTE and len(self.arm_joint_indices) < 6:
                self.arm_joint_indices.append(j)

            # Gripper finger joints
            if "finger" in joint_name.lower():
                self.finger_joint_indices.append(j)

            # End effector frame (use link with 'flange' or last arm link)
            if "flange" in link_name.lower() or "wrist3" in link_name.lower():
                self.grasp_frame_index = j

        # Use last arm joint if no flange found
        if self.grasp_frame_index is None and self.arm_joint_indices:
            self.grasp_frame_index = self.arm_joint_indices[-1]

        print(f"Arm joints: {len(self.arm_joint_indices)}")
        print(f"Finger joints: {len(self.finger_joint_indices)}")
        print(f"Grasp frame index: {self.grasp_frame_index}")

        # Get joint limits
        self.joint_limits = []
        for j in self.arm_joint_indices:
            info = p.getJointInfo(self.robot_id, j)
            lo, hi = float(info[8]), float(info[9])
            if lo > hi or abs(lo) > 10 or abs(hi) > 10:
                lo, hi = -2.8, 2.8
            self.joint_limits.append((lo, hi))

        # Create target block
        self.target_pos = np.array([0.5, -0.25, self.tabletop_z + 0.025])
        self.target_id = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025],
                                                     rgbaColor=[0.1, 0.2, 0.8, 1]),
            basePosition=self.target_pos.tolist()
        )

        # Create obstacle
        self.obstacle_pos = np.array([0.5, 0., self.tabletop_z + 0.15])
        self.obstacle_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.3),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.3,
                                                     rgbaColor=[0.8, 0.2, 0.1, 1]),
            basePosition=self.obstacle_pos.tolist()
        )

        # Obstacle list for collision checking
        self.obstacle_ids = [self.obstacle_id, self.table]

        # Home configuration (safer, collision-free position - arm lifted up and away from obstacle)
        self.home_config = [0, -0.8, 0.8, -2.0, -1.57, 0]

        # Create simple gripper if no gripper joints found
        if not self.finger_joint_indices:
            print("No gripper joints found in URDF. Creating simple gripper...")
            self._create_simple_gripper()

        self._reset_to_home()

    def _create_simple_gripper(self):
        """Create a simple parallel gripper attached to the end effector"""
        # Get end effector position
        ee_state = p.getLinkState(self.robot_id, self.grasp_frame_index)
        ee_pos = ee_state[0]
        ee_orn = ee_state[1]

        # Create gripper base (palm)
        palm_size = [0.04, 0.02, 0.01]
        self.gripper_palm = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=palm_size),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=palm_size,
                                                     rgbaColor=[0.3, 0.3, 0.3, 1]),
            basePosition=[ee_pos[0], ee_pos[1], ee_pos[2] - 0.05]
        )

        # Create two finger objects
        finger_size = [0.01, 0.005, 0.04]
        self.gripper_finger_left = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=finger_size),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=finger_size,
                                                     rgbaColor=[0.2, 0.2, 0.2, 1]),
            basePosition=[ee_pos[0], ee_pos[1] + 0.02, ee_pos[2] - 0.09]
        )

        self.gripper_finger_right = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=finger_size),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=finger_size,
                                                     rgbaColor=[0.2, 0.2, 0.2, 1]),
            basePosition=[ee_pos[0], ee_pos[1] - 0.02, ee_pos[2] - 0.09]
        )

        self.gripper_width = 0.04  # Current gripper opening
        self.gripper_attached = True
        print(f"Simple gripper created and attached to end effector")

    def _update_gripper_position(self):
        """Update gripper position to follow end effector"""
        if not hasattr(self, 'gripper_attached') or not self.gripper_attached:
            return

        ee_state = p.getLinkState(self.robot_id, self.grasp_frame_index)
        ee_pos = ee_state[0]
        ee_orn = ee_state[1]

        # Update palm position
        p.resetBasePositionAndOrientation(
            self.gripper_palm,
            [ee_pos[0], ee_pos[1], ee_pos[2] - 0.05],
            ee_orn
        )

        # Update finger positions (symmetric around center)
        half_width = self.gripper_width / 2
        p.resetBasePositionAndOrientation(
            self.gripper_finger_left,
            [ee_pos[0], ee_pos[1] + half_width, ee_pos[2] - 0.09],
            ee_orn
        )
        p.resetBasePositionAndOrientation(
            self.gripper_finger_right,
            [ee_pos[0], ee_pos[1] - half_width, ee_pos[2] - 0.09],
            ee_orn
        )

    def _reset_to_home(self):
        """Reset robot to home configuration smoothly"""
        print("Resetting to home position...")

        # First, gently move joints to home using motor control (no sudden jumps)
        for _ in range(120):
            p.setJointMotorControlArray(
                self.robot_id, self.arm_joint_indices, p.POSITION_CONTROL,
                targetPositions=self.home_config,
                targetVelocities=[0.0]*len(self.arm_joint_indices),
                positionGains=[0.05]*len(self.arm_joint_indices),  # Lower gain for smoother motion
                velocityGains=[0.1]*len(self.arm_joint_indices),
                forces=[80]*len(self.arm_joint_indices),
            )

            # Update gripper position
            if hasattr(self, 'gripper_attached') and self.gripper_attached:
                self._update_gripper_position()

            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        # Open gripper
        if self.finger_joint_indices:
            for joint_id in self.finger_joint_indices:
                p.resetJointState(self.robot_id, joint_id, 0.04, targetVelocity=0.0)
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL,
                                       targetPosition=0.04, force=60)
        elif hasattr(self, 'gripper_attached') and self.gripper_attached:
            self.gripper_width = 0.08  # Open gripper

        # Stabilize
        for _ in range(50):
            if hasattr(self, 'gripper_attached') and self.gripper_attached:
                self._update_gripper_position()
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        print("Home position reached")

    def _get_arm_q(self):
        """Get current arm joint angles in radians"""
        return np.array([p.getJointState(self.robot_id, j)[0] for j in self.arm_joint_indices], dtype=float)

    def _exec_to_q(self, q_target, duration=0.5, pos_gain=0.3, vel_gain=0.8, force=200):
        """
        Smoothly move arm to target configuration using minimum-jerk trajectory.

        Args:
            q_target: Target joint configuration (radians)
            duration: Motion duration in seconds (default: 0.5s)
            pos_gain: Position PID gain (tuned from 0.12 to 0.3)
            vel_gain: Velocity PID gain (tuned from 0.35 to 0.8)
            force: Maximum force in Newtons (tuned from 170 to 200)
        """
        q_current = self._get_arm_q()
        num_steps = int(duration / DT)  # Calculate steps based on duration

        # Generate minimum-jerk trajectory for smooth motion
        trajectory = TrajectoryInterpolator.minimum_jerk(q_current, q_target, num_steps)

        # Execute trajectory with tuned PID gains
        for q in trajectory:
            p.setJointMotorControlArray(
                self.robot_id, self.arm_joint_indices, p.POSITION_CONTROL,
                targetPositions=[float(x) for x in q],
                targetVelocities=[0.0]*len(self.arm_joint_indices),
                positionGains=[pos_gain]*len(self.arm_joint_indices),
                velocityGains=[vel_gain]*len(self.arm_joint_indices),
                forces=[force]*len(self.arm_joint_indices)
            )

            # Update gripper position to follow end effector
            if hasattr(self, 'gripper_attached') and self.gripper_attached:
                self._update_gripper_position()

            p.stepSimulation()
            time.sleep(DT if GUI else 0)

    def open_gripper(self, duration=1.0):
        """Open the gripper with smooth motion"""
        print("Opening gripper...")

        if self.finger_joint_indices:
            # Get current gripper position
            current_pos = p.getJointState(self.robot_id, self.finger_joint_indices[0])[0]
            target_pos = 0.04  # Fully open

            # Generate smooth trajectory
            num_steps = int(duration / DT)
            trajectory = TrajectoryInterpolator.minimum_jerk(
                np.array([current_pos]),
                np.array([target_pos]),
                num_steps
            )

            # Execute smooth gripper motion
            for pos_array in trajectory:
                pos = float(pos_array[0])
                for joint_id in self.finger_joint_indices:
                    p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL,
                                           targetPosition=pos, force=60, maxVelocity=0.5)
                p.stepSimulation()
                time.sleep(DT if GUI else 0)

        elif hasattr(self, 'gripper_attached') and self.gripper_attached:
            # Simple gripper - animate opening
            num_steps = int(duration / DT)
            current_width = self.gripper_width
            target_width = 0.08  # 8cm open

            for i in range(num_steps):
                alpha = (i + 1) / num_steps
                self.gripper_width = current_width + alpha * (target_width - current_width)
                self._update_gripper_position()
                p.stepSimulation()
                time.sleep(DT if GUI else 0)

    def close_gripper(self, duration=1.0):
        """Close the gripper with smooth motion"""
        print("Closing gripper...")

        if self.finger_joint_indices:
            # Get current gripper position
            current_pos = p.getJointState(self.robot_id, self.finger_joint_indices[0])[0]
            target_pos = 0.0  # Fully closed

            # Generate smooth trajectory
            num_steps = int(duration / DT)
            trajectory = TrajectoryInterpolator.minimum_jerk(
                np.array([current_pos]),
                np.array([target_pos]),
                num_steps
            )

            # Execute smooth gripper motion with contact detection
            for pos_array in trajectory:
                pos = float(pos_array[0])
                for joint_id in self.finger_joint_indices:
                    p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL,
                                           targetPosition=pos, force=100, maxVelocity=0.5)
                p.stepSimulation()
                time.sleep(DT if GUI else 0)

                # Check for contact (optional: stop if object grasped)
                contacts = p.getContactPoints(bodyA=self.robot_id, linkIndexA=self.finger_joint_indices[0])
                if len(contacts) > 0 and any(c[9] > 5.0 for c in contacts):  # 5N threshold
                    print(f"  Contact detected at {pos*2000:.1f}mm")
                    break

        elif hasattr(self, 'gripper_attached') and self.gripper_attached:
            # Simple gripper - animate closing
            num_steps = int(duration / DT)
            current_width = self.gripper_width
            target_width = 0.01  # 1cm closed (for grasping)

            for i in range(num_steps):
                alpha = (i + 1) / num_steps
                self.gripper_width = current_width + alpha * (target_width - current_width)
                self._update_gripper_position()
                p.stepSimulation()
                time.sleep(DT if GUI else 0)

                # Check for contact with target object
                if hasattr(self, 'target_id'):
                    contacts_left = p.getContactPoints(bodyA=self.gripper_finger_left, bodyB=self.target_id)
                    contacts_right = p.getContactPoints(bodyA=self.gripper_finger_right, bodyB=self.target_id)
                    if len(contacts_left) > 0 or len(contacts_right) > 0:
                        print(f"  Contact detected - object grasped!")
                        break

    # ========================= RRT-CONNECT PATH PLANNING =========================

    def _in_collision(self, q, self_collisions=False):
        """Check if configuration q causes collision"""
        original_states = [(p.getJointState(self.robot_id, j)[0], p.getJointState(self.robot_id, j)[1])
                           for j in self.arm_joint_indices]
        collision_found = False

        try:
            for j, val in zip(self.arm_joint_indices, q):
                p.resetJointState(self.robot_id, j, float(val))

            if self_collisions:
                pts = p.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id)
                if len(pts) > 0:
                    collision_found = True

            if not collision_found:
                for oid in self.obstacle_ids:
                    pts = p.getClosestPoints(bodyA=self.robot_id, bodyB=oid, distance=0.0)
                    if len(pts) > 0:
                        collision_found = True
                        break
        finally:
            for j, (pos, vel) in zip(self.arm_joint_indices, original_states):
                p.resetJointState(self.robot_id, j, pos, targetVelocity=vel)

        return collision_found

    def _edge_collision_free(self, q_a, q_b, num_checks=25):
        """Check if path between q_a and q_b is collision-free"""
        for s in np.linspace(0.0, 1.0, num_checks):
            q = (1 - s) * q_a + s * q_b
            if self._in_collision(q, self_collisions=False):
                return False
        return True

    def _sample_q(self):
        """Sample random configuration within joint limits"""
        return np.array([random.uniform(lo, hi) for (lo, hi) in self.joint_limits], dtype=float)

    def _steer(self, q_from, q_to, step_size):
        """Steer from q_from toward q_to with max distance step_size"""
        v = q_to - q_from
        d = float(np.linalg.norm(v))
        if d < 1e-12:
            return q_to.copy()
        return q_from + (v / d) * min(step_size, d)

    def _nearest(self, tree_q, q):
        """Find nearest node in tree to q"""
        dmin = float("inf")
        imin = 0
        for i, tq in enumerate(tree_q):
            d = float(np.linalg.norm(q - tq))
            if d < dmin:
                dmin = d
                imin = i
        return imin

    def rrt_connect(self, q_start, q_goal, step_size=0.12, max_iters=4000, goal_bias=0.20):
        """RRT-Connect bidirectional planning"""
        Ta_q, Ta_parent = [q_start.copy()], [-1]
        Tb_q, Tb_parent = [q_goal.copy()], [-1]

        def extend(Tq, Tparent, q_target):
            i_near = self._nearest(Tq, q_target)
            q_near = Tq[i_near]
            q_new = self._steer(q_near, q_target, step_size)
            if not self._edge_collision_free(q_near, q_new, num_checks=15):
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

    def shortcut_path(self, path, tries=250):
        """Smooth path by removing unnecessary waypoints"""
        if path is None or len(path) < 3:
            return path
        path = [q.copy() for q in path]
        shortcuts_made = 0

        for _ in range(tries):
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if j - i < 2:
                continue
            if self._edge_collision_free(path[i], path[j], num_checks=25):
                path = path[:i+1] + path[j:]
                shortcuts_made += 1

        print(f"Shortcutting removed {shortcuts_made} waypoints")
        return path

    def execute_path(self, path, duration_per_waypoint=0.125):
        """
        Execute a planned path with smooth minimum-jerk interpolation.

        Args:
            path: List of joint configurations (waypoints)
            duration_per_waypoint: Duration for each waypoint segment (default: 0.125s)
        """
        # Add minimum-jerk interpolation between waypoints for even smoother motion
        print(f"Interpolating path with {len(path)} waypoints using minimum-jerk...")
        interpolated_path = []

        for i in range(len(path) - 1):
            # Generate smooth minimum-jerk trajectory between consecutive waypoints
            segment = TrajectoryInterpolator.minimum_jerk(path[i], path[i + 1], num_points=15)
            if i == 0:
                interpolated_path.extend(segment)
            else:
                # Skip first point to avoid duplication
                interpolated_path.extend(segment[1:])

        path = np.array(interpolated_path)
        print(f"Interpolated to {len(path)} points for ultra-smooth motion")

        # Execute interpolated path
        print(f"Executing smooth path...")
        for i, q in enumerate(path):
            self._exec_to_q(q, duration=duration_per_waypoint)

            # Update gripper to follow end effector
            if hasattr(self, 'gripper_attached') and self.gripper_attached:
                self._update_gripper_position()

            if i % 50 == 0:
                print(f"  Waypoint {i}/{len(path)}")

    def run_pick_and_place(self):
        """Execute complete pick-and-place task"""
        print("=" * 60)
        print("FR5 Robot Pick-and-Place with RRT-Connect")
        print("=" * 60)

        time.sleep(1)

        # Define end-effector orientation (face down)
        ee_euler = (math.pi, 0.0, 0.0)
        ee_quat = p.getQuaternionFromEuler(ee_euler)

        # ========== PHASE 1: Move to above target ==========
        print("\n[1/7] Planning path to above target...")
        z_mid = self.tabletop_z + 0.40
        above_target = np.array([self.target_pos[0], self.target_pos[1], z_mid])

        q_start = self._get_arm_q()
        theta_above = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, above_target.tolist(), ee_quat,
            solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
        )
        q_goal = np.array(theta_above[:len(self.arm_joint_indices)], dtype=float)

        path = self.rrt_connect(q_start, q_goal, step_size=0.06, max_iters=2000, goal_bias=0.25)
        if path is None:
            print("RRT failed, using direct motion instead...")
            path = [q_start, q_goal]
        else:
            path = self.shortcut_path(path, tries=300)
        # Interpolate for smoothness
        interpolated = [path[0]]
        for i in range(len(path) - 1):
            for alpha in np.linspace(0.01, 0.99, 70):
                interpolated.append((1 - alpha) * path[i] + alpha * path[i + 1])
            interpolated.append(path[i + 1])

        self.execute_path(interpolated)
        print("Reached above target")

        # ========== PHASE 2: Descend to grasp ==========
        print("\n[2/7] Descending to grasp...")
        grasp_pose = np.array([self.target_pos[0], self.target_pos[1], self.target_pos[2] + 0.005])

        for z in np.linspace(z_mid, grasp_pose[2] - 0.003, 70):
            wp = np.array([self.target_pos[0], self.target_pos[1], float(z)])
            theta = p.calculateInverseKinematics(
                self.robot_id, self.grasp_frame_index, wp.tolist(), ee_quat,
                solver=p.IK_DLS, maxNumIterations=400, residualThreshold=1e-4
            )
            q_tgt = np.array(theta[:len(self.arm_joint_indices)], dtype=float)
            self._exec_to_q(q_tgt, duration=0.1)

        for _ in range(100):
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        # ========== PHASE 3: Close gripper ==========
        print("\n[3/7] Grasping object...")
        self.close_gripper()

        # ========== PHASE 4: Lift object ==========
        print("\n[4/7] Lifting object...")
        lifted_pos = (self.target_pos + np.array([0, 0, 0.25])).tolist()
        theta_lift = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, lifted_pos, ee_quat,
            solver=p.IK_DLS, maxNumIterations=200, residualThreshold=1e-4
        )
        p.setJointMotorControlArray(
            self.robot_id, self.arm_joint_indices, p.POSITION_CONTROL,
            targetPositions=theta_lift[:len(self.arm_joint_indices)],
            positionGains=[0.1]*len(self.arm_joint_indices),
            velocityGains=[0.5]*len(self.arm_joint_indices),
            forces=[150]*len(self.arm_joint_indices)
        )
        for _ in range(100):
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        # ========== PHASE 5: Transport via arch over obstacle ==========
        print("\n[5/7] Planning transport path over obstacle...")
        place_pos = np.array([0.5, 0.25, self.tabletop_z + 0.025])
        place_above = np.array([place_pos[0], place_pos[1], z_mid])
        arch_point = np.array([self.obstacle_pos[0], self.obstacle_pos[1], z_mid + 0.05])

        # Plan to arch
        q_current = self._get_arm_q()
        theta_arch = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, arch_point.tolist(), ee_quat,
            solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
        )
        q_arch = np.array(theta_arch[:len(self.arm_joint_indices)], dtype=float)

        path1 = self.rrt_connect(q_current, q_arch, step_size=0.06, max_iters=2000, goal_bias=0.25)
        if path1 is None:
            print("RRT to arch failed, using direct motion...")
            path1 = [q_current, q_arch]
        else:
            path1 = self.shortcut_path(path1, tries=300)

        # Plan from arch to place
        theta_place = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, place_above.tolist(), ee_quat,
            solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
        )
        q_place = np.array(theta_place[:len(self.arm_joint_indices)], dtype=float)

        path2 = self.rrt_connect(q_arch, q_place, step_size=0.06, max_iters=2000, goal_bias=0.25)
        if path2 is None:
            print("RRT to place failed, using direct motion...")
            path2 = [q_arch, q_place]
        else:
            path2 = self.shortcut_path(path2, tries=300)

        # Combine and interpolate
        if path1 and path2:
            combined = path1 + path2[1:]
            interpolated = [combined[0]]
            for i in range(len(combined) - 1):
                for alpha in np.linspace(0.01, 0.99, 70):
                    interpolated.append((1 - alpha) * combined[i] + alpha * combined[i + 1])
                interpolated.append(combined[i + 1])

            self.execute_path(interpolated, duration_per_waypoint=0.25)
            print("Reached above place position")

        # ========== PHASE 6: Descend and release ==========
        print("\n[6/7] Placing object...")
        for z in np.linspace(z_mid, place_pos[2] - 0.003, 70):
            wp = np.array([place_pos[0], place_pos[1], float(z)])
            theta = p.calculateInverseKinematics(
                self.robot_id, self.grasp_frame_index, wp.tolist(), ee_quat,
                solver=p.IK_DLS, maxNumIterations=400, residualThreshold=1e-4
            )
            q_tgt = np.array(theta[:len(self.arm_joint_indices)], dtype=float)
            self._exec_to_q(q_tgt, duration=0.1)

        for _ in range(100):
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        self.open_gripper()

        # Lift after placing
        lifted_after = np.array([place_pos[0], place_pos[1], z_mid])
        theta_lift_after = p.calculateInverseKinematics(
            self.robot_id, self.grasp_frame_index, lifted_after.tolist(), ee_quat,
            solver=p.IK_DLS, maxNumIterations=200, residualThreshold=1e-4
        )
        p.setJointMotorControlArray(
            self.robot_id, self.arm_joint_indices, p.POSITION_CONTROL,
            targetPositions=theta_lift_after[:len(self.arm_joint_indices)],
            positionGains=[0.1]*len(self.arm_joint_indices),
            velocityGains=[0.5]*len(self.arm_joint_indices),
            forces=[150]*len(self.arm_joint_indices)
        )
        for _ in range(100):
            p.stepSimulation()
            time.sleep(DT if GUI else 0)

        # ========== PHASE 7: Return home ==========
        print("\n[7/7] Returning home...")
        self.obstacle_ids.append(self.target_id)  # Add placed cube as obstacle

        q_current = self._get_arm_q()
        q_home = np.array(self.home_config, dtype=float)

        home_path = self.rrt_connect(q_current, q_home, step_size=0.06, max_iters=2000, goal_bias=0.25)
        if home_path is None:
            print("RRT to home failed, using direct motion...")
            home_path = [q_current, q_home]
        else:
            home_path = self.shortcut_path(home_path, tries=300)

        interpolated = [home_path[0]]
        for i in range(len(home_path) - 1):
            for alpha in np.linspace(0.01, 0.99, 70):
                interpolated.append((1 - alpha) * home_path[i] + alpha * home_path[i + 1])
            interpolated.append(home_path[i + 1])

        self.execute_path(interpolated, duration_per_waypoint=0.125)
        print("Returned home")

        print("\n" + "=" * 60)
        print("✓ Pick-and-place task completed successfully!")
        print("=" * 60)


def main():
    robot = FR5PickAndPlace(gui=GUI)

    try:
        robot.run_pick_and_place()
    except Exception as e:
        import traceback
        print("\n=== ERROR ===")
        print(f"Error: {e}")
        traceback.print_exc()
        print("=============\n")

    print("\nSimulation running... Close window to exit.")
    if GUI:
        while True:
            p.stepSimulation()
            time.sleep(DT)


if __name__ == "__main__":
    main()
