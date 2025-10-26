#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        self.trajectory_pub = self.create_publisher(Marker, 'trajectory_path', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'tomato/goal_pose',
            self.pose_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Storage for trajectory
        self.trajectory_points = []
        self.marker_id = 0
        
        self.get_logger().info('Visualization Node initialized')
    
    def pose_callback(self, msg):
        '''Visualize detected tomato pose'''
        
        marker_array = MarkerArray()
        
        # Create sphere marker at tomato location
        sphere_marker = Marker()
        sphere_marker.header = msg.header
        sphere_marker.ns = "tomato_detections"
        sphere_marker.id = self.marker_id
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        
        sphere_marker.pose = msg.pose
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 0.8
        
        sphere_marker.lifetime.sec = 5  # Display for 5 seconds
        
        marker_array.markers.append(sphere_marker)
        
        # Create text label
        text_marker = Marker()
        text_marker.header = msg.header
        text_marker.ns = "tomato_labels"
        text_marker.id = self.marker_id + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose = msg.pose
        text_marker.pose.position.z += 0.1  # Offset above sphere
        
        text_marker.scale.z = 0.03
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = f"Tomato\n({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})"
        text_marker.lifetime.sec = 5
        
        marker_array.markers.append(text_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)
        
        # Add to trajectory
        self.trajectory_points.append(msg.pose.position)
        self.publish_trajectory()
        
        self.marker_id += 2
        self.get_logger().info(f'Visualized tomato at pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})')
    
    def joint_callback(self, msg):
        '''Visualize current joint states'''
        # Joint states are automatically visualized via robot_state_publisher
        # This callback can be used for custom visualizations if needed
        pass
    
    def publish_trajectory(self):
        '''Publish trajectory line showing all detected tomato positions'''
        
        if len(self.trajectory_points) < 2:
            return
        
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "base_link"
        trajectory_marker.header.stamp = self.get_clock().now().to_msg()
        trajectory_marker.ns = "trajectory"
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        
        trajectory_marker.scale.x = 0.01  # Line width
        
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 1.0
        trajectory_marker.color.b = 0.0
        trajectory_marker.color.a = 0.5
        
        # Add all points to line strip
        trajectory_marker.points = self.trajectory_points[-20:]  # Last 20 points
        
        self.trajectory_pub.publish(trajectory_marker)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
