from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the path to the config file
    config = os.path.join(
        get_package_share_directory('agrirover_perception'),
        'config',
        'camera_params.yaml'
    )
    
    # Declare launch arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='webcam',
        description='Camera type: webcam or realsense'
    )
    
    z_distance_arg = DeclareLaunchArgument(
        'z_distance',
        default_value='0.5',
        description='Default Z distance for webcam (meters)'
    )
    
    # Camera node
    camera_node = Node(
        package='agrirover_perception',
        executable='camera_node',
        name='tomato_detection_camera_node',
        parameters=[config, {
            'camera_type': LaunchConfiguration('camera_type'),
            'z_distance': LaunchConfiguration('z_distance'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_type_arg,
        z_distance_arg,
        camera_node,
    ])
