#!/usr/bin/env python3
"""
Enhanced Manual Goal Pose Publisher
- Preset positions
- Coordinate validation
- Movement history
- Better UI
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque


class EnhancedManualGoalPosePublisher(Node):
    def __init__(self):
        super().__init__('manual_goal_pose_publisher')
        
        # Declare parameters
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('marker_lifetime', 10.0)
        self.declare_parameter('history_size', 10)
        
        # Presets (name: [x, y, z])
        self.presets = {
            'home': [0.0, 0.0, 0.3],
            'pick1': [0.25, 0.15, 0.2],
            'pick2': [0.3, -0.1, 0.18],
            'drop': [0.0, 0.3, 0.25],
            }

        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.history_size = self.get_parameter('history_size').value        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'tomato/goal_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'goal_pose_markers', 10)
        
        # History
        self.pose_history = deque(maxlen=self.history_size)
        self.marker_id_counter = 0
        
        self.print_welcome()
        self.run_interactive_loop()
    
    def print_welcome(self):
        """Print welcome message and instructions"""
        print('═' * 70)
        print('  ENHANCED MANUAL GOAL POSE PUBLISHER')
        print('═' * 70)
        print(f'Publishing to: tomato/goal_pose')
        print(f'Frame: {self.frame_id}')
        print()
        print('USAGE:')
        print('  Enter coordinates:     x y z')
        print('  Example:               0.3 0.2 0.1')
        print()
        print('PRESETS:')
        for name, pos in self.presets.items():
            print(f'  {name:<15} [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')
        print()
        print('COMMANDS:')
        print('  history     - Show recent poses')
        print('  repeat N    - Repeat pose from history (N = index)')
        print('  clear       - Clear visualization markers')
        print('  quit/exit   - Exit program')
        print('═' * 70)
    
    def publish_goal_pose(self, x, y, z, label="Manual"):
        """Publish goal pose and marker"""
        # Create pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)
        
        # Add to history
        self.pose_history.append((x, y, z, label))
        
        # Create marker
        self.publish_marker(x, y, z, label)
        
        print(f'✓ Published: [{x:.3f}, {y:.3f}, {z:.3f}] ({label})')
    
    def publish_marker(self, x, y, z, label):
        """Publish visualization marker"""
        marker_array = MarkerArray()
        
        # Sphere marker
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "goal_poses"
        marker.id = self.marker_id_counter
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime = rclpy.duration.Duration(
            seconds=0
        ).to_msg()
        
        marker_array.markers.append(marker)
        
        # Text marker
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "goal_labels"
        text_marker.id = self.marker_id_counter
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = z + 0.08
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.03
        
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = label
        text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        
        marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        self.marker_id_counter += 2
    
    def show_history(self):
        """Display pose history"""
        if not self.pose_history:
            print('  (No history)')
            return
        
        print('\nHISTORY:')
        for i, (x, y, z, label) in enumerate(self.pose_history):
            print(f'  [{i}] {label:<15} [{x:.3f}, {y:.3f}, {z:.3f}]')
        print()
    
    def clear_markers(self):
        """Clear all visualization markers"""
        marker_array = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        self.marker_pub.publish(marker_array)
        self.marker_id_counter = 0
        
        print('✓ Markers cleared')
    
    def run_interactive_loop(self):
        """Interactive input loop"""
        while rclpy.ok():
            try:
                user_input = input('\n> ').strip()
                
                if not user_input:
                    continue
                
                # Handle commands
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print('Exiting...')
                    break
                
                if user_input.lower() == 'history':
                    self.show_history()
                    continue
                
                if user_input.lower() == 'clear':
                    self.clear_markers()
                    continue
                
                if user_input.lower().startswith('repeat'):
                    parts = user_input.split()
                    if len(parts) == 2:
                        try:
                            idx = int(parts[1])
                            if 0 <= idx < len(self.pose_history):
                                x, y, z, label = self.pose_history[idx]
                                self.publish_goal_pose(x, y, z, f"Repeat-{label}")
                            else:
                                print(f'✗ Index {idx} out of range')
                        except ValueError:
                            print('✗ Invalid index')
                    continue
                
                # Check presets
                if user_input.lower() in self.presets:
                    pos = self.presets[user_input.lower()]
                    self.publish_goal_pose(pos[0], pos[1], pos[2], user_input)
                    continue
                
                # Parse coordinates
                parts = user_input.split()
                if len(parts) != 3:
                    print('✗ Invalid input. Expected: x y z')
                    continue
                
                try:
                    x, y, z = map(float, parts)
                    self.publish_goal_pose(x, y, z)
                except ValueError:
                    print('✗ Invalid numbers')
                
            except KeyboardInterrupt:
                print('\nExiting...')
                break
            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnhancedManualGoalPosePublisher()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
