"""
RRT-Connect bidirectional path planner for FR5 in PyBullet.

Extracted from python_sdk/simulation/first_example.py.
Standalone class — no simulation setup, just planning logic.
"""

import numpy as np
import random
import pybullet as p


class RRTPlanner:
    """
    RRT-Connect bidirectional path planner for FR5 in PyBullet.

    Usage:
        planner = RRTPlanner(robot_id, arm_joint_indices, joint_limits, obstacle_ids)
        path = planner.rrt_connect(start_config, goal_config)
        smooth = planner.shortcut_path(path)
    """

    def __init__(self, robot_id, arm_joint_indices, joint_limits, obstacle_ids,
                 step_size=0.10, max_iters=4000, goal_bias=0.15,
                 shortcut_iterations=400, edge_checks=20, collision_margin=0.02):
        """
        Args:
            robot_id: PyBullet body ID of the robot
            arm_joint_indices: List of joint indices for the 6 arm joints
            joint_limits: List of (lo, hi) tuples in radians for each joint
            obstacle_ids: List of PyBullet body IDs to check collisions against
            step_size: RRT step size in radians (default 0.10)
            max_iters: Max RRT iterations per tree (default 4000)
            goal_bias: Probability of sampling the goal directly (default 0.15)
            shortcut_iterations: Shortcut smoothing iterations (default 400)
            edge_checks: Number of collision checks per edge segment (default 20)
            collision_margin: Contact distance threshold for collision (default 0.02m)
        """
        self.robot_id = robot_id
        self.arm_joint_indices = arm_joint_indices
        self.joint_limits = joint_limits
        self.obstacle_ids = obstacle_ids
        self.step_size = step_size
        self.max_iters = max_iters
        self.goal_bias = goal_bias
        self.shortcut_iterations = shortcut_iterations
        self.edge_checks = edge_checks
        self.collision_margin = collision_margin

    def _random_config(self):
        """Sample a random joint configuration within joint limits."""
        return [random.uniform(lo, hi) for lo, hi in self.joint_limits]

    def _nearest(self, tree, q):
        """Find nearest node in tree to configuration q (Euclidean in joint space)."""
        dists = [np.linalg.norm(np.array(node) - np.array(q)) for node in tree]
        return tree[np.argmin(dists)]

    def _steer(self, q_from, q_to):
        """Step from q_from toward q_to by at most step_size."""
        diff = np.array(q_to) - np.array(q_from)
        dist = np.linalg.norm(diff)
        if dist <= self.step_size:
            return list(q_to)
        return list(np.array(q_from) + (diff / dist) * self.step_size)

    def in_collision(self, config):
        """
        Check if robot in config collides with any obstacle.

        Saves and restores joint states so the simulation state is unchanged.

        Returns:
            True if collision detected, False if clear.
        """
        # Save current joint states
        saved = [p.getJointState(self.robot_id, j)[0] for j in self.arm_joint_indices]

        # Set joints to query config
        for j, angle in zip(self.arm_joint_indices, config):
            p.resetJointState(self.robot_id, j, angle)

        p.performCollisionDetection()

        collision = False
        for obs_id in self.obstacle_ids:
            contacts = p.getContactPoints(self.robot_id, obs_id)
            if contacts:
                for contact in contacts:
                    if contact[8] < -self.collision_margin:
                        collision = True
                        break
            if collision:
                break

        # Restore original joint states
        for j, angle in zip(self.arm_joint_indices, saved):
            p.resetJointState(self.robot_id, j, angle)

        return collision

    def edge_free(self, q1, q2):
        """
        Check if straight-line path from q1 to q2 is collision-free.

        Interpolates edge_checks points along the segment.

        Returns:
            True if all intermediate configs are collision-free.
        """
        for i in range(self.edge_checks + 1):
            t = i / self.edge_checks
            q = [q1[j] + t * (q2[j] - q1[j]) for j in range(len(q1))]
            if self.in_collision(q):
                return False
        return True

    def _extend(self, tree, q_rand):
        """
        Try to extend tree toward q_rand.

        Returns:
            (status, q_new) where status is 'reached', 'advanced', or 'trapped'
        """
        q_near = self._nearest(tree, q_rand)
        q_new = self._steer(q_near, q_rand)

        if self.in_collision(q_new):
            return 'trapped', None

        if not self.edge_free(q_near, q_new):
            return 'trapped', None

        tree.append(q_new)

        if np.linalg.norm(np.array(q_new) - np.array(q_rand)) < 1e-6:
            return 'reached', q_new
        return 'advanced', q_new

    def _connect(self, tree, q_target):
        """
        Repeatedly extend tree toward q_target until reached or trapped.

        Returns:
            'reached' if connected, 'trapped' otherwise
        """
        status = 'advanced'
        while status == 'advanced':
            status, _ = self._extend(tree, q_target)
        return status

    def rrt_connect(self, start, goal):
        """
        Bidirectional RRT-Connect from start to goal.

        Args:
            start: Start joint configuration (radians)
            goal: Goal joint configuration (radians)

        Returns:
            List of joint configs (path) from start to goal, or None if failed.
        """
        if self.in_collision(start):
            return None
        if self.in_collision(goal):
            return None

        tree_a = [list(start)]
        tree_b = [list(goal)]
        parent_a = {0: None}
        parent_b = {0: None}

        for iteration in range(self.max_iters):
            # Sample random config (with goal bias toward tree_b tip)
            if random.random() < self.goal_bias:
                q_rand = list(tree_b[-1])
            else:
                q_rand = self._random_config()

            # Extend tree_a toward q_rand
            status_a, q_new_a = self._extend(tree_a, q_rand)

            if status_a != 'trapped':
                idx_a = len(tree_a) - 1
                parent_a[idx_a] = len(tree_a) - 2

                # Connect tree_b toward q_new_a
                status_b = self._connect(tree_b, q_new_a)

                if status_b == 'reached':
                    # Trees connected — extract path
                    path_a = self._extract_path(tree_a, parent_a, len(tree_a) - 1)
                    path_b = self._extract_path(tree_b, parent_b, len(tree_b) - 1)
                    return path_a + list(reversed(path_b[:-1]))

                idx_b = len(tree_b) - 1
                parent_b[idx_b] = len(tree_b) - 2

            # Swap trees for alternating growth
            tree_a, tree_b = tree_b, tree_a
            parent_a, parent_b = parent_b, parent_a

        return None  # Failed to find path

    def _extract_path(self, tree, parent, leaf_idx):
        """Trace path from root to leaf through parent pointers."""
        path = []
        idx = leaf_idx
        while idx is not None:
            path.append(tree[idx])
            idx = parent.get(idx)
        return list(reversed(path))

    def shortcut_path(self, path):
        """
        Shorten path by attempting direct edges between non-adjacent nodes.

        Args:
            path: List of joint configs

        Returns:
            Smoothed path (fewer waypoints).
        """
        if path is None or len(path) <= 2:
            return path

        smooth = list(path)
        for _ in range(self.shortcut_iterations):
            if len(smooth) <= 2:
                break
            i = random.randint(0, len(smooth) - 2)
            j = random.randint(i + 1, len(smooth) - 1)
            if self.edge_free(smooth[i], smooth[j]):
                smooth = smooth[:i + 1] + smooth[j:]

        return smooth
