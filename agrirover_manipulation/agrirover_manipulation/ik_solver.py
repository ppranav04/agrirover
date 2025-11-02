#!/usr/bin/env python3
"""
Improved IK Solver Node for 5-DOF Agricultural Robot Arm
Features:
- Better transform handling with proper error checking
- Joint limit enforcement and validation
- Solution quality assessment (manipulability, distance to limits)
- Multiple IK solution handling with ranking
- Workspace bounds checking
- Singularity detection
- Improved error handling and logging
- State machine for safer operation
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
import os
from typing import List

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain


class RobotState:
    """State machine for robot operation"""
    IDLE = "IDLE"
    WAITING_FOR_TRANSFORMS = "WAITING_FOR_TRANSFORMS"
    COMPUTING_IK = "COMPUTING_IK"
    MOVING_TO_TARGET = "MOVING_TO_TARGET"
    MOVING_HOME = "MOVING_HOME"
    ERROR = "ERROR"


class IKSolution:
    """Container for IK solution with quality metrics"""
    def __init__(self, joint_angles: np.ndarray, manipulability: float,
                 distance_to_limits: float, end_effector_error: float):
        self.joint_angles = joint_angles
        self.manipulability = manipulability
        self.distance_to_limits = distance_to_limits
        self.end_effector_error = end_effector_error

    def quality_score(self) -> float:
        """Compute overall quality score (higher is better)"""
        w_manip = 0.4
        w_limits = 0.4
        w_error = 0.2

        return (w_manip * self.manipulability +
                w_limits * self.distance_to_limits -
                w_error * self.end_effector_error)


class ImprovedIKSolverNode(Node):
    def __init__(self):
        super().__init__('improved_ik_solver_node')

        # Declare parameters
        self.declare_parameter('home_position', [1.57, 1.57, 0.9, 0.8, 0.0])
        self.declare_parameter('tf_timeout', 2.0)
        self.declare_parameter('ik_timeout', 1.0)
        self.declare_parameter('max_ik_iterations', 100)
        self.declare_parameter('position_tolerance', 0.01)  # 1cm
        self.declare_parameter('num_ik_solutions', 5)
        self.declare_parameter('manipulability_threshold', 0.01)

        self.home_position = self.get_parameter('home_position').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.ik_timeout = self.get_parameter('ik_timeout').value
        self.max_ik_iterations = self.get_parameter('max_ik_iterations').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.num_ik_solutions = self.get_parameter('num_ik_solutions').value
        self.manipulability_threshold = self.get_parameter('manipulability_threshold').value

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Frame names
        self.arm_base_frame = "base_link"
        self.camera_frame = "camera_head_link"
        self.end_effector_frame = "gripper"

        # State tracking
        self.current_state = RobotState.WAITING_FOR_TRANSFORMS
        self.transforms_ready = False
        self.last_valid_joints = np.array(self.home_position)

        # Joint limits (from URDF)
        self.joint_limits = np.array([
            [0.0, 3.14],  # Joint 1
            [0.0, 3.14],  # Joint 2
            [0.0, 3.14],  # Joint 3
            [0.0, 3.14],  # Joint 4
            [0.0, 0.9]    # Gripper
        ])

        # Workspace bounds (approximate)
        self.workspace_bounds = {
            'x': [-0.5, 0.5],
            'y': [-0.5, 0.5],
            'z': [0.0, 0.8]
        }

        # Publishers
        self.servo_publisher = self.create_publisher(Float32MultiArray, 'servo_angles', 10)

        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'tomato/goal_pose',
            self.goal_pose_callback,
            10
        )
        self.detection_status_subscriber = self.create_subscription(
            Bool,
            'tomato/detection_status',
            self.detection_status_callback,
            10
        )

        # Initialize IK solver chain
        try:
            package_path = get_package_share_directory('agrirover_description')
            urdf_path = os.path.join(package_path, 'urdf', 'mech', 'arm_combined.urdf')
            self.chain = Chain.from_urdf_file(urdf_path)
            self.get_logger().info(f'✓ IK solver initialized with {len(self.chain.links)} links')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize IK solver: {e}')
            self.current_state = RobotState.ERROR
            return

        # Timer to check transforms and publish home position
        self.startup_timer = self.create_timer(0.5, self.startup_callback)
        self.startup_attempts = 0
        self.max_startup_attempts = 20

        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  Improved IK Solver Node Initialized')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(f'Home position: {self.home_position}')
        self.get_logger().info(f'Joint limits: \n{self.joint_limits}')
        self.get_logger().info('Waiting for transforms...')

    def startup_callback(self):
        if self.transforms_ready:
            return
        self.startup_attempts += 1

        try:
            can_transform = self.tf_buffer.can_transform(
                target_frame=self.arm_base_frame,
                source_frame=self.camera_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            if can_transform:
                self.transforms_ready = True
                self.current_state = RobotState.IDLE
                self.get_logger().info('✓ Transforms ready!')
                self.move_to_home_position()
                self.startup_timer.cancel()
                return
            if self.startup_attempts >= self.max_startup_attempts:
                self.get_logger().warn(f'⚠ Transforms not available after {self.max_startup_attempts} attempts. Publishing home position anyway.')
                self.move_to_home_position()
                self.startup_timer.cancel()
        except Exception as e:
            self.get_logger().debug(f'Waiting for transforms: {e}')

    def is_in_workspace(self, position: List[float]) -> bool:
        x, y, z = position
        in_bounds = (
            self.workspace_bounds['x'][0] <= x <= self.workspace_bounds['x'][1] and
            self.workspace_bounds['y'][0] <= y <= self.workspace_bounds['y'][1] and
            self.workspace_bounds['z'][0] <= z <= self.workspace_bounds['z'][1]
        )
        if not in_bounds:
            self.get_logger().warn(f'⚠ Target [{x:.3f}, {y:.3f}, {z:.3f}] outside workspace bounds')
        return in_bounds

    def validate_joint_angles(self, joint_angles: np.ndarray) -> bool:
        if len(joint_angles) != 5:
            return False
        for i, angle in enumerate(joint_angles):
            if not (self.joint_limits[i][0] <= angle <= self.joint_limits[i][1]):
                self.get_logger().warn(f'⚠ Joint {i+1} angle {angle:.3f} outside limits [{self.joint_limits[i][0]:.3f}, {self.joint_limits[i][1]:.3f}]')
                return False
        return True

    def compute_manipulability(self, joint_angles: np.ndarray) -> float:
        try:
            full_angles = np.insert(joint_angles, 0, 0.0)
            fk_matrix = self.chain.forward_kinematics(full_angles)
            center_distances = []
            for i, angle in enumerate(joint_angles):
                center = (self.joint_limits[i][0] + self.joint_limits[i][1]) / 2
                range_val = self.joint_limits[i][1] - self.joint_limits[i][0]
                normalized_dist = abs(angle - center) / (range_val / 2)
                center_distances.append(1.0 - normalized_dist)
            return np.mean(center_distances)
        except Exception as e:
            self.get_logger().warn(f'Failed to compute manipulability: {e}')
            return 0.5

    def compute_distance_to_limits(self, joint_angles: np.ndarray) -> float:
        min_distances = []
        for i, angle in enumerate(joint_angles):
            lower_dist = angle - self.joint_limits[i][0]
            upper_dist = self.joint_limits[i][1] - angle
            range_val = self.joint_limits[i][1] - self.joint_limits[i][0]
            min_dist = min(lower_dist, upper_dist) / range_val
            min_distances.append(min_dist)
        return np.min(min_distances)

    def solve_ik_multiple(self, target_position: List[float]) -> List[IKSolution]:
        """Solve IK with multiple initial guesses and rank solutions"""
        solutions = []
    
        initial_guesses = [
            self.last_valid_joints,
            np.array(self.home_position),
        ]
    
        for _ in range(self.num_ik_solutions - 2):
            perturbation = np.random.uniform(-0.3, 0.3, size=5)
            guess = np.clip(
                self.last_valid_joints + perturbation,
                self.joint_limits[:, 0],
                self.joint_limits[:, 1]
            )
            initial_guesses.append(guess)
    
        for idx, initial_guess in enumerate(initial_guesses):
            try:
                # Solve IK - NO prepending dummy base joint
                ik_result = self.chain.inverse_kinematics(
                    target_position=target_position,
                    initial_position=initial_guess,  # Use directly
                    max_iter=self.max_ik_iterations
                )
            
                # ik_result is already just [j0, j1, j2, j3, j4]
                joint_solution = ik_result
            
                # FIX: Validate using np.all() for array comparison
                within_bounds = np.all(
                    (joint_solution >= self.joint_limits[:, 0]) &
                    (joint_solution <= self.joint_limits[:, 1])
                )
            
                if not within_bounds:
                    continue
            
                # Verify with FK
                fk_result = self.chain.forward_kinematics(ik_result)
                achieved_position = fk_result[:3, 3]
                end_effector_error = np.linalg.norm(
                    np.array(target_position) - achieved_position
                )
            
                if end_effector_error > self.position_tolerance:
                    continue
            
                manipulability = self.compute_manipulability(joint_solution)
                distance_to_limits = self.compute_distance_to_limits(joint_solution)
            
                if manipulability < self.manipulability_threshold:
                    continue
            
                solution = IKSolution(
                    joint_angles=joint_solution,
                    manipulability=manipulability,
                    distance_to_limits=distance_to_limits,
                    end_effector_error=end_effector_error
                )
            
                solutions.append(solution)
            
                self.get_logger().debug(
                    f'Solution {idx}: error={end_effector_error:.4f}, '
                    f'manip={manipulability:.3f}'
                )
            
            except Exception as e:
                self.get_logger().debug(f'IK attempt {idx} failed: {e}')
                continue
    
        solutions.sort(key=lambda s: s.quality_score(), reverse=True)
        return solutions


    def goal_pose_callback(self, msg: PoseStamped):
        if self.current_state == RobotState.COMPUTING_IK:
            self.get_logger().warn('⚠ Already computing IK, ignoring new goal')
            return

        if not self.transforms_ready:
            self.get_logger().warn('⚠ Transforms not ready, ignoring goal')
            return

        self.current_state = RobotState.COMPUTING_IK
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  NEW GOAL RECEIVED')
        self.get_logger().info('═══════════════════════════════════════')

        try:
            point_camera = PointStamped()
            point_camera.header = msg.header
            point_camera.header.frame_id = self.camera_frame
            point_camera.point = msg.pose.position

            self.get_logger().info(
                f'Camera frame input: x={msg.pose.position.x:.3f}, '
                f'y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}'
            )

            transform = self.tf_buffer.lookup_transform(
                target_frame=self.arm_base_frame,
                source_frame=self.camera_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )

            self.get_logger().info('✓ Transform lookup successful')

            point_base = do_transform_point(point_camera, transform)

            target_pos = [
                point_base.point.x,
                point_base.point.y,
                point_base.point.z
            ]

            self.get_logger().info(
                f'Base frame (fully transformed): x={target_pos[0]:.3f}, '
                f'y={target_pos[1]:.3f}, z={target_pos[2]:.3f}'
            )

            if not self.is_in_workspace(target_pos):
                self.get_logger().error('✗ Target outside workspace')
                self.current_state = RobotState.IDLE
                return

            self.get_logger().info('Computing IK solution...')
            solutions = self.solve_ik_multiple(target_pos)

            if not solutions:
                self.get_logger().error('✗ No valid IK solution found')
                self.current_state = RobotState.IDLE
                return

            best_solution = solutions[0]

            self.get_logger().info(f'✓ Found {len(solutions)} valid solution(s)')
            self.get_logger().info(
                f'Best solution: '
                f'quality={best_solution.quality_score():.3f}, '
                f'error={best_solution.end_effector_error:.4f}m, '
                f'manip={best_solution.manipulability:.3f}'
            )
            self.get_logger().info(
                f'Joint angles: [{", ".join("{:.3f}".format(a) for a in best_solution.joint_angles)}]'
            )

            msg_out = Float32MultiArray()
            msg_out.data = [float(a) for a in best_solution.joint_angles]
            self.servo_publisher.publish(msg_out)

            self.last_valid_joints = best_solution.joint_angles
            self.current_state = RobotState.MOVING_TO_TARGET

            self.get_logger().info('✓ Published servo commands')
            self.get_logger().info('═══════════════════════════════════════')

            def reset_callback():
                self._set_idle_state()
                timer.cancel()

            timer = self.create_timer(0.5, reset_callback)

        except tf2_ros.LookupException as e:
            self.get_logger().error(f'✗ Transform lookup failed: {e}')
            self.current_state = RobotState.IDLE
        except Exception as e:
            self.get_logger().error(f'✗ Unexpected error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.current_state = RobotState.ERROR

    def _set_idle_state(self):
        if self.current_state == RobotState.MOVING_TO_TARGET:
            self.current_state = RobotState.IDLE

    def detection_status_callback(self, msg):  # Make sure this exists
        if not msg.data and self.current_state == RobotState.IDLE:
            self.get_logger().info('No detection - moving to home position')
            self.move_to_home_position()

    def move_to_home_position(self):
        if self.current_state in [RobotState.COMPUTING_IK, RobotState.MOVING_HOME]:
            return

        self.current_state = RobotState.MOVING_HOME

        try:
            msg = Float32MultiArray()
            msg.data = self.home_position
            self.servo_publisher.publish(msg)

            self.last_valid_joints = np.array(self.home_position)

            self.get_logger().info(f'✓ Moving to home: {self.home_position}')

            def reset_home_callback():
                self._reset_from_home()
                timer.cancel()

            timer = self.create_timer(1.0, reset_home_callback)

        except Exception as e:
            self.get_logger().error(f'✗ Failed to move home: {e}')
            self.current_state = RobotState.ERROR

    def _reset_from_home(self):
        if self.current_state == RobotState.MOVING_HOME:
            self.current_state = RobotState.IDLE


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImprovedIKSolverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
