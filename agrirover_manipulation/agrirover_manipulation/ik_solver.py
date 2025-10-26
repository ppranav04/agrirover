#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from ikpy.chain import Chain
from ikpy.link import URDFLink, OriginLink, Link


# IK Solver class (your full IKSolver5DOF_IKPy definition)
class IKSolver5DOF_IKPy:
    """
    5 DOF IK Solver using IKPy library.
    Simple and effective for quick prototyping.
    """
    def __init__(self, urdf_file=None, links=None, active_links_mask=None):
        if urdf_file is not None:
            self.chain = Chain.from_urdf_file(urdf_file, active_links_mask=active_links_mask)
            print(self.chain)
        elif links is not None:
            self.chain = Chain(links, active_links_mask=active_links_mask)
        else:
            raise ValueError("Either urdf_file or links must be provided")
    
    def solve_ik(self, target_position, target_orientation=None, initial_position=None, orientation_mode=None):
        if target_orientation is not None:
            ik_result = self.chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation,
                initial_position=initial_position,
                orientation_mode=orientation_mode)
        else:
            ik_result = self.chain.inverse_kinematics(
                target_position=target_position,
                initial_position=initial_position)
        return ik_result
    
    def forward_kinematics(self, joint_angles):
        return self.chain.forward_kinematics(joint_angles)

    @staticmethod
    def create_simple_5dof_chain():
        links = [
            OriginLink(),
            Link(name="joint_1", bounds=(-np.pi, np.pi), axis=[0, 0, 1], translation=[0,0,0.1]),
            Link(name="joint_2", bounds=(0, np.pi/2), axis=[0, 1, 0], translation=[0,0,0]),
            Link(name="joint_3", bounds=(-np.pi, np.pi), axis=[0, 1, 0], translation=[0.3,0,0]),
            Link(name="joint_4", bounds=(-np.pi, np.pi), axis=[0, 1, 0], translation=[0.25,0,0]),
            Link(name="joint_5", bounds=(-np.pi, np.pi), axis=[1, 0, 0], translation=[0,0,0]),
        ]
        return Chain(links, name="5DOF_Arm")


class IkSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        # Create publisher to 'servo_angles' topic
        self.publisher_ = self.create_publisher(Float32MultiArray, 'servo_angles', 10)

        # Subscribe to tomato goal pose from camera node
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'tomato/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Initialize IK solver
        package_path = get_package_share_directory('agrirover_description')
        urdf_path = os.path.join(package_path, 'urdf', 'mech', 'arm_mech.urdf')
        self.solver = IKSolver5DOF_IKPy(urdf_file=urdf_path, active_links_mask=[False, True, True, True, True, True])

        self.get_logger().info('IK Solver Node initialized and waiting for goal poses...')

    def goal_pose_callback(self, msg: PoseStamped):
        '''Callback when goal pose is received from camera node'''
        
        # Extract target position [x, y, z]
        target_pos = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        
        self.get_logger().info(f'Received goal pose: x={target_pos[0]:.3f}, y={target_pos[1]:.3f}, z={target_pos[2]:.3f}')

        # Compute IK
        try:
            joint_angles_full = self.solver.solve_ik(target_pos)
            joint_angles = joint_angles_full[1:6]  # Extract joints 1-5
            
            # Create and populate ROS message
            msg = Float32MultiArray()
            msg.data = joint_angles.tolist()

            self.get_logger().info(f"Publishing joint angles: {msg.data}")

            # Publish joint angles to serial_motor_commander
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'IK solution failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IkSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
