#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher_node')
        
        # Publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber to servo angles
        self.servo_sub = self.create_subscription(
            Float32MultiArray,
            'servo_angles',
            self.servo_callback,
            10
        )
        
        self.get_logger().info('Robot State Publisher initialized')
    
    def servo_callback(self, msg):
        '''Convert servo angles to JointState and publish'''
        
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names (must match URDF)
        joint_state.name = ['base_to_link_0', 'link_0_to_link_1', 'link_1_to_link_2', 'link_2_to_gripper', 'finger_1_joint']
        
        # Positions in radians (already in radians from IK solver)
        joint_state.position = list(msg.data)
        
        # Velocities (optional)
        #joint_state.velocity = [0.0] * len(msg.data)
        
        # Efforts (optional)
        #joint_state.effort = [0.0] * len(msg.data)
        
        self.joint_state_pub.publish(joint_state)
        
        self.get_logger().info(f'Published joint states: {joint_state.position}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
