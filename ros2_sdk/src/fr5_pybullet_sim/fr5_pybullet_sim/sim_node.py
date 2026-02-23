#!/usr/bin/env python3
"""
FR5 PyBullet Simulation Node with RRT-Connect Path Planning

This ROS 2 node provides:
- PyBullet physics simulation of the FR5 robot
- Joint state publishing on /joint_states
- TCP pose publishing on /tcp_pose
- Service to set joint positions: /fr5_sim/set_joint_position
- Service to move with path planning: /fr5_sim/move_to_joint
- RRT-Connect collision-free path planning
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import os
import random
import threading
from ament_index_python.packages import get_package_share_directory
try:
    from scipy.interpolate import CubicSpline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class FR5PyBulletSim(Node):
    def __init__(self):
        super().__init__('fr5_pybullet_sim')

        # Declare parameters
        self.gui = self.declare_parameter('gui', True).value
        self.urdf_name = self.declare_parameter('urdf_name', 'fairino5_v6.urdf').value
        self.sim_rate = self.declare_parameter('sim_rate', 120.0).value

        self.get_logger().info(f"Starting FR5 PyBullet Simulation (GUI: {self.gui})")

        # PyBullet setup
        self.physics_client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(1.0 / self.sim_rate)
        p.resetDebugVisualizerCamera(2.0, 45, -30, [0.5, 0, 0.5])
        p.setGravity(0, 0, -9.81)

        # Load environment
        self.plane_id = p.loadURDF("plane.urdf")
        self.table_xyz = [0.5, 0, 0]
        self.table = p.loadURDF("table/table.urdf", self.table_xyz)
        self.tabletop_z = p.getAABB(self.table)[1][2]

        # Find and load FR5 URDF
        urdf_path = self._find_urdf()
        if urdf_path is None:
            self.get_logger().error("Could not find robot URDF!")
            raise RuntimeError("URDF not found")

        self.get_logger().info(f"Loading URDF: {urdf_path}")

        base_xyz = [0.0, 0.0, self.tabletop_z + 0.001]
        base_orientation = p.getQuaternionFromEuler([0, 0, math.pi])
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=base_xyz,
            baseOrientation=base_orientation,
            useFixedBase=True
        )

        # Find joint info
        self.joint_names = []
        self.arm_joint_indices = []
        self.end_effector_index = None

        num_joints = p.getNumJoints(self.robot_id)
        for j in range(num_joints):
            info = p.getJointInfo(self.robot_id, j)
            joint_type = info[2]
            joint_name = info[1].decode()
            link_name = info[12].decode()

            if joint_type == p.JOINT_REVOLUTE:
                self.joint_names.append(joint_name)
                self.arm_joint_indices.append(j)

            if "wrist3" in link_name.lower() or "link6" in link_name.lower():
                self.end_effector_index = j

        if self.end_effector_index is None and self.arm_joint_indices:
            self.end_effector_index = self.arm_joint_indices[-1]

        self.num_joints = len(self.arm_joint_indices)
        self.get_logger().info(f"Found {self.num_joints} revolute joints: {self.joint_names}")

        # Get joint limits
        self.joint_limits = []
        for j in self.arm_joint_indices:
            info = p.getJointInfo(self.robot_id, j)
            lo, hi = float(info[8]), float(info[9])
            if lo > hi or abs(lo) > 10 or abs(hi) > 10:
                lo, hi = -2.8, 2.8
            self.joint_limits.append((lo, hi))

        # Add obstacles for collision checking
        self.target_pos = np.array([0.5, -0.25, self.tabletop_z + 0.025])
        self.target_id = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025],
                                                     rgbaColor=[0.1, 0.2, 0.8, 1]),
            basePosition=self.target_pos.tolist()
        )

        self.obstacle_pos = np.array([0.5, 0., self.tabletop_z + 0.15])
        self.obstacle_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.3),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.3,
                                                     rgbaColor=[0.8, 0.2, 0.1, 1]),
            basePosition=self.obstacle_pos.tolist()
        )

        self.obstacle_ids = [self.obstacle_id, self.table]

        # Set home configuration and reset
        self.home_config = [0, -0.5, 0.5, -1.5, 0, 1.0][:self.num_joints]
        self._reset_to_home()

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tcp_pose_pub = self.create_publisher(PoseStamped, '/tcp_pose', 10)

        # Services
        self.home_srv = self.create_service(Trigger, '/fr5_sim/go_home', self.go_home_callback)
        self.target_srv = self.create_service(Trigger, '/fr5_sim/go_to_target', self.go_to_target_callback)
        self.place_srv = self.create_service(Trigger, '/fr5_sim/go_to_place', self.go_to_place_callback)

        # Motion state
        self.is_moving = False
        self.motion_lock = threading.Lock()

        # Timer for simulation update
        timer_period = 1.0 / 50.0  # 50 Hz for publishing
        self.timer = self.create_timer(timer_period, self.update_callback)

        self.get_logger().info("FR5 PyBullet Simulation Node started!")
        self.get_logger().info("Services available:")
        self.get_logger().info("  - /fr5_sim/go_home")
        self.get_logger().info("  - /fr5_sim/go_to_target")
        self.get_logger().info("  - /fr5_sim/go_to_place")

    def _find_urdf(self):
        """Find the robot URDF file"""
        # Try to find in fairino_description package
        try:
            pkg_share = get_package_share_directory('fairino_description')
            urdf_path = os.path.join(pkg_share, 'urdf', self.urdf_name)
            if os.path.exists(urdf_path):
                return urdf_path
        except Exception:
            pass

        # Try local paths (relative to script location)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        possible_paths = [
            os.path.join(script_dir, '..', '..', '..', 'fairino_description', 'urdf', self.urdf_name),
            os.path.join(script_dir, '..', '..', 'simulation', 'urdf', 'fairino5_v6.urdf'),
            os.path.join(script_dir, '..', '..', '..', 'src', 'fairino_description', 'urdf', self.urdf_name),
            os.path.join(script_dir, '..', '..', 'urdf', self.urdf_name),
        ]

        for path in possible_paths:
            if os.path.exists(path):
                return os.path.abspath(path)

        return None

    def _reset_to_home(self):
        """Reset robot to home configuration"""
        for joint_id, theta in zip(self.arm_joint_indices, self.home_config):
            p.resetJointState(self.robot_id, joint_id, theta, targetVelocity=0.0)

        p.setJointMotorControlArray(
            self.robot_id, self.arm_joint_indices, p.POSITION_CONTROL,
            targetPositions=self.home_config,
            targetVelocities=[0.0] * self.num_joints,
            positionGains=[0.08] * self.num_joints,
            velocityGains=[0.2] * self.num_joints,
            forces=[90] * self.num_joints,
        )

        for _ in range(100):
            p.stepSimulation()

    def _get_arm_q(self):
        """Get current arm joint angles in radians"""
        return np.array([p.getJointState(self.robot_id, j)[0] for j in self.arm_joint_indices], dtype=float)

    # ========================= RRT-CONNECT PATH PLANNING =========================

    def _in_collision(self, q, self_collisions=False):
        """Check if configuration q causes collision with obstacles."""
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
        """Check if path between q_a and q_b is collision-free."""
        for s in np.linspace(0.0, 1.0, num_checks):
            q = (1 - s) * q_a + s * q_b
            if self._in_collision(q, self_collisions=False):
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

    def rrt_connect(self, q_start, q_goal, step_size=0.12, max_iters=4000, goal_bias=0.20):
        """RRT-Connect bidirectional planning."""
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
                self.get_logger().info(f"RRT-Connect succeeded at iteration {iteration}")
                return path_a + path_b[1:]

            Ta_q, Tb_q = Tb_q, Ta_q
            Ta_parent, Tb_parent = Tb_parent, Ta_parent

        self.get_logger().warn("RRT-Connect failed: max iterations reached")
        return None

    def shortcut_path(self, path, tries=250):
        """Smooth path by removing unnecessary waypoints."""
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
            if self._edge_collision_free(path[i], path[j], num_checks=25):
                path = path[:i + 1] + path[j:]
                shortcuts_made += 1

        self.get_logger().info(f"Shortcutting removed {shortcuts_made} waypoints")
        return path

    def smooth_path_spline(self, path, num_output_points=100):
        """
        Smooth path using cubic splines for continuous curvature.
        Uses clamped boundary conditions (zero velocity at endpoints).
        """
        if not HAS_SCIPY or path is None or len(path) < 3:
            return path

        path_array = np.array(path)
        n_joints = path_array.shape[1]
        t_knots = np.linspace(0, 1, len(path))
        t_output = np.linspace(0, 1, num_output_points)

        smoothed = np.zeros((num_output_points, n_joints))
        for j in range(n_joints):
            # Clamped spline: zero velocity at start and end
            cs = CubicSpline(t_knots, path_array[:, j], bc_type='clamped')
            smoothed[:, j] = cs(t_output)

        return [smoothed[i] for i in range(num_output_points)]

    def minimum_jerk_interpolate(self, path, points_per_segment=15):
        """
        Apply minimum-jerk interpolation between waypoints for smooth motion.
        Formula: s(t) = 10t^3 - 15t^4 + 6t^5 ensures zero velocity/acceleration at endpoints.
        """
        if path is None or len(path) < 2:
            return path

        result = [path[0].copy()]
        for i in range(len(path) - 1):
            q_start = np.array(path[i])
            q_end = np.array(path[i + 1])

            for j in range(1, points_per_segment + 1):
                t = j / points_per_segment
                # Minimum-jerk polynomial
                s = 10 * t**3 - 15 * t**4 + 6 * t**5
                q = q_start + (q_end - q_start) * s
                result.append(q)

        return result

    def _exec_to_q(self, q_target, steps=10):
        """Smoothly move arm to target configuration."""
        p.setJointMotorControlArray(
            self.robot_id, self.arm_joint_indices, p.POSITION_CONTROL,
            targetPositions=[float(x) for x in q_target],
            targetVelocities=[0.0] * self.num_joints,
            positionGains=[0.12] * self.num_joints,
            velocityGains=[0.35] * self.num_joints,
            forces=[170] * self.num_joints
        )
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(1.0 / self.sim_rate)

    def move_to_joint_positions(self, target_radians):
        """Move robot to target joint positions using RRT-Connect"""
        q_goal = np.array(target_radians, dtype=float)
        q_start = self._get_arm_q()

        if self._in_collision(q_goal):
            self.get_logger().warn("Goal configuration is in collision!")
            return False

        self.get_logger().info("Starting RRT-Connect planning...")
        path = self.rrt_connect(q_start, q_goal, step_size=0.06, max_iters=5000, goal_bias=0.25)

        if path is None:
            self.get_logger().warn("RRT-Connect failed. Attempting direct motion...")
            # Direct motion fallback
            for _ in range(200):
                self._exec_to_q(q_goal, steps=10)
            return True

        self.get_logger().info(f"Initial path length: {len(path)} waypoints")
        path = self.shortcut_path(path, tries=500)
        self.get_logger().info(f"After shortcutting: {len(path)} waypoints")

        # Apply B-spline smoothing for continuous curvature (if scipy available)
        if HAS_SCIPY and len(path) >= 3:
            path = self.smooth_path_spline(path, num_output_points=max(50, len(path) * 3))
            self.get_logger().info(f"After spline smoothing: {len(path)} waypoints")

        # Apply minimum-jerk interpolation for smooth velocity profile
        path = self.minimum_jerk_interpolate(path, points_per_segment=10)
        self.get_logger().info(f"Final trajectory: {len(path)} waypoints")

        self.get_logger().info(f"Executing path with {len(path)} waypoints...")
        for i, q in enumerate(path):
            self._exec_to_q(q, steps=10)
            if i % 50 == 0:
                self.get_logger().info(f"  Waypoint {i}/{len(path)}")

        self.get_logger().info("Move completed!")
        return True

    # ========================= SERVICE CALLBACKS =========================

    def go_home_callback(self, request, response):
        """Service callback to return robot to home position"""
        self.get_logger().info("Received go_home request")

        with self.motion_lock:
            if self.is_moving:
                response.success = False
                response.message = "Robot is already moving"
                return response

            self.is_moving = True

        try:
            success = self.move_to_joint_positions(self.home_config)
            response.success = success
            response.message = "Moved to home" if success else "Failed to move"
        finally:
            with self.motion_lock:
                self.is_moving = False

        return response

    def go_to_target_callback(self, request, response):
        """Service callback to move robot above target block"""
        self.get_logger().info("Received go_to_target request")

        with self.motion_lock:
            if self.is_moving:
                response.success = False
                response.message = "Robot is already moving"
                return response

            self.is_moving = True

        try:
            # Calculate IK for target position
            z_above = self.tabletop_z + 0.35
            target_above = [self.target_pos[0], self.target_pos[1], z_above]
            ee_quat = p.getQuaternionFromEuler([math.pi, 0.0, 0.0])

            theta = p.calculateInverseKinematics(
                self.robot_id, self.end_effector_index, target_above, ee_quat,
                solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
            )

            # Enforce joint limits on IK result
            q_goal = np.array(theta[:self.num_joints])
            for j, (lo, hi) in enumerate(self.joint_limits):
                q_goal[j] = np.clip(q_goal[j], lo, hi)

            success = self.move_to_joint_positions(q_goal.tolist())
            response.success = success
            response.message = "Moved to target" if success else "Failed to move"
        except Exception as e:
            self.get_logger().error(f"IK error: {e}")
            response.success = False
            response.message = str(e)
        finally:
            with self.motion_lock:
                self.is_moving = False

        return response

    def go_to_place_callback(self, request, response):
        """Service callback to move robot to place position (other side of obstacle)"""
        self.get_logger().info("Received go_to_place request")

        with self.motion_lock:
            if self.is_moving:
                response.success = False
                response.message = "Robot is already moving"
                return response

            self.is_moving = True

        try:
            # Calculate IK for place position
            z_above = self.tabletop_z + 0.35
            place_pos = [0.5, 0.25, z_above]
            ee_quat = p.getQuaternionFromEuler([math.pi, 0.0, 0.0])

            theta = p.calculateInverseKinematics(
                self.robot_id, self.end_effector_index, place_pos, ee_quat,
                solver=p.IK_DLS, maxNumIterations=500, residualThreshold=1e-4
            )

            # Enforce joint limits on IK result
            q_goal = np.array(theta[:self.num_joints])
            for j, (lo, hi) in enumerate(self.joint_limits):
                q_goal[j] = np.clip(q_goal[j], lo, hi)

            success = self.move_to_joint_positions(q_goal.tolist())
            response.success = success
            response.message = "Moved to place position" if success else "Failed to move"
        except Exception as e:
            self.get_logger().error(f"IK error: {e}")
            response.success = False
            response.message = str(e)
        finally:
            with self.motion_lock:
                self.is_moving = False

        return response

    # ========================= UPDATE CALLBACK =========================

    def update_callback(self):
        """Timer callback to step simulation and publish states"""
        # Step simulation
        p.stepSimulation()

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = []
        joint_state.velocity = []
        joint_state.effort = []

        for idx in self.arm_joint_indices:
            state = p.getJointState(self.robot_id, idx)
            joint_state.position.append(state[0])
            joint_state.velocity.append(state[1])
            joint_state.effort.append(state[3])

        self.joint_state_pub.publish(joint_state)

        # Publish TCP pose
        if self.end_effector_index is not None:
            link_state = p.getLinkState(self.robot_id, self.end_effector_index)
            pos = link_state[0]
            orn = link_state[1]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = joint_state.header.stamp
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = pos[2]
            pose_msg.pose.orientation.x = orn[0]
            pose_msg.pose.orientation.y = orn[1]
            pose_msg.pose.orientation.z = orn[2]
            pose_msg.pose.orientation.w = orn[3]
            self.tcp_pose_pub.publish(pose_msg)

        # Hold current positions against gravity when not moving
        with self.motion_lock:
            is_currently_moving = self.is_moving

        if not is_currently_moving:
            p.setJointMotorControlArray(
                bodyUniqueId=self.robot_id,
                jointIndices=self.arm_joint_indices,
                controlMode=p.POSITION_CONTROL,
                targetPositions=joint_state.position,
                forces=[500.0] * self.num_joints
            )


def main(args=None):
    rclpy.init(args=args)
    node = FR5PyBulletSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        p.disconnect()  # Disconnect PyBullet last (after ROS2 shutdown)


if __name__ == '__main__':
    main()
