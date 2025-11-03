#!/usr/bin/env python3
"""
Enhanced Robot State Publisher Node (FIXED)
- Pin 5 removed: 4 servos instead of 5
- Publishes joint states from servo commands
- Initializes TF tree with home position
- Adds velocity and effort estimation
- Better error handling
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np
from collections import deque



class EnhancedRobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher_node')
        
        # Declare parameters
        # FIXED: 4 joints instead of 5 (pin 5 removed)
        self.declare_parameter('home_position', [1.57, 0.9, 0.8, 0.0])
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('joint_names', [
            'base_to_link_0', 
            'link_0_to_link_1', 
            'link_1_to_link_2', 
            'link_2_to_gripper', 
        ])
        
        # Get parameters
        self.home_position = self.get_parameter('home_position').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joint_names = self.get_parameter('joint_names').value
        
        # Validate: should have 4 joints
        if len(self.home_position) != 4:
            self.get_logger().error(
                f'Home position must have 4 values, got {len(self.home_position)}'
            )
            raise ValueError("Invalid home_position length")
        
        if len(self.joint_names) != 4:
            self.get_logger().error(
                f'Joint names must have 4 values, got {len(self.joint_names)}'
            )
            raise ValueError("Invalid joint_names length")
        
        # State tracking
        self.current_positions = np.array(self.home_position)
        self.previous_positions = np.array(self.home_position)
        self.current_velocities = np.zeros(len(self.home_position))
        self.position_history = deque(maxlen=10)
        self.last_update_time = self.get_clock().now()
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', 10
        )
        
        # Subscribers
        self.servo_sub = self.create_subscription(
            Float32MultiArray,
            'servo_angles',
            self.servo_callback,
            10
        )
        
        # Timer for regular publishing
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_joint_state
        )
        
        # Publish initial state immediately
        self.publish_joint_state()
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  Enhanced Robot State Publisher')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(f'Servos: 4 (pin 5 removed)')
        self.get_logger().info(f'Joint names: {self.joint_names}')
        self.get_logger().info(f'Home position: {self.home_position}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
    
    def servo_callback(self, msg: Float32MultiArray):
        """Update current positions from servo commands"""
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'Received {len(msg.data)} angles, expected 4'
            )
            return
        
        # Store previous position
        self.previous_positions = self.current_positions.copy()
        
        # Update current position
        self.current_positions = np.array(msg.data)
        
        # Estimate velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if dt > 0:
            self.current_velocities = (
                self.current_positions - self.previous_positions
            ) / dt
        
        self.last_update_time = current_time
        
        # Add to history
        self.position_history.append(self.current_positions.copy())
        
        self.get_logger().debug(
            f'Updated positions: {self.current_positions}'
        )
    
    def publish_joint_state(self):
        """Publish current joint state"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_positions.tolist()
        joint_state.velocity = self.current_velocities.tolist()
        joint_state.effort = [0.0] * 4  # Unknown
        
        self.joint_state_pub.publish(joint_state)



def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnhancedRobotStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
