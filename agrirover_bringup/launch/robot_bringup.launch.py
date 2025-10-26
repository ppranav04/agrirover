from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Camera Node
        Node(
            package='agrirover_perception',
            executable='camera_node',
            name='tomato_detection_camera_node',
            parameters=[{
                'camera_type': 'webcam',  # or 'realsense'
                'camera_id': 0,
                'z_distance': 0.5,
                'model_path': '/home/pranav/agrirover_ws/src/yolov8n_tomato.pt'
            }],
            output='screen'
        ),
        
        # IK Solver Node
        Node(
            package='agrirover_manipulation',
            executable='ik_solver',
            name='ik_solver_node',
            output='screen'
        ),
        
        # Serial Motor Commander Node
        Node(
            package='agrirover_manipulation',
            executable='serial_motor_commander',
            name='serial_motor_commander',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 9600
            }],
            output='screen'
        ),
    ])
