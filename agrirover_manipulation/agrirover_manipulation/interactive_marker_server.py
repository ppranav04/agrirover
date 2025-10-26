#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import Header
import math

class RobotInteractiveMarkerServer(Node):
    def __init__(self):
        super().__init__('robot_interactive_marker_server')
        
        # Create interactive marker server
        self.server = InteractiveMarkerServer(self, 'robot_control_marker')
        
        # Publisher for target pose to IK solver
        self.pose_publisher = self.create_publisher(
            PoseStamped, 
            '/ik_solver/target_pose', 
            10
        )
        
        # Create the interactive marker
        self.make_6dof_marker()
        
        # Apply changes to show the marker
        self.server.applyChanges()
        
        self.get_logger().info('Interactive Marker Server started')
    
    def make_6dof_marker(self, fixed=False):
        """
        Create a 6DOF interactive marker for controlling the robot end-effector
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = 'robot_end_effector'
        int_marker.description = 'Robot End-Effector Control\n6-DOF Control'
        int_marker.scale = 0.3
        
        # Set initial pose
        int_marker.pose.position.x = 0.5
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.5
        int_marker.pose.orientation.w = 1.0
        
        # Create a visible marker (sphere) for the control point
        self.make_box_control(int_marker)
        
        # Add 6DOF controls
        if fixed:
            int_marker.description += '\n(fixed orientation)'
        
        # Move and rotate controls
        self.add_6dof_controls(int_marker, fixed)
        
        # Insert marker with feedback callback
        self.server.insert(
            int_marker,
            feedback_callback=self.process_feedback
        )
    
    def make_box_control(self, msg):
        """
        Create a visible marker (sphere) to represent the end-effector position
        """
        control = InteractiveMarkerControl()
        control.always_visible = True
        
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.8
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0
        
        control.markers.append(marker)
        msg.controls.append(control)
    
    def add_6dof_controls(self, int_marker, fixed_orientation):
        """
        Add 6DOF (translation + rotation) controls to the interactive marker
        """
        control = InteractiveMarkerControl()
        
        if fixed_orientation:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        
        # X-axis controls (Red)
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # Y-axis controls (Green)
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # Z-axis controls (Blue)
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
    
    def process_feedback(self, feedback):
        """
        Callback function to handle interactive marker feedback
        Publishes the target pose to the IK solver
        """
        # Log the feedback
        if feedback.event_type == 4:  # Pose Update
            self.get_logger().info(
                f'Marker pose updated:\n'
                f'Position: x={feedback.pose.position.x:.3f}, '
                f'y={feedback.pose.position.y:.3f}, '
                f'z={feedback.pose.position.z:.3f}\n'
                f'Orientation: x={feedback.pose.orientation.x:.3f}, '
                f'y={feedback.pose.orientation.y:.3f}, '
                f'z={feedback.pose.orientation.z:.3f}, '
                f'w={feedback.pose.orientation.w:.3f}'
            )
            
            # Create PoseStamped message for IK solver
            target_pose = PoseStamped()
            target_pose.header = feedback.header
            target_pose.pose = feedback.pose
            
            # Publish to IK solver
            self.pose_publisher.publish(target_pose)
        
        elif feedback.event_type == InteractiveMarkerControl.BUTTON_CLICK:
            self.get_logger().info('Marker clicked')
        
        elif feedback.event_type == InteractiveMarkerControl.MENU_SELECT:
            self.get_logger().info(f'Menu item {feedback.menu_entry_id} clicked')
        
        # Apply changes to update the marker
        self.server.applyChanges()

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotInteractiveMarkerServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
