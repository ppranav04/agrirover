from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

robot_urdf_path = os.path.join(
    get_package_share_directory('agrirover_description'),
    'urdf',
    'mech',
    'arm_mech.urdf'
)

with open(robot_urdf_path, 'r') as f:
    robot_description_xml = f.read()

def generate_launch_description():

    # Get the path to the camera config file
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
        default_value='0.1',
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
        # Camera Node
        camera_type_arg,
        z_distance_arg,
        camera_node,
        
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
            output='screen'
        ),
        
        # Visualization Node
        Node(
            package='agrirover_manipulation',
            executable='visualization_node',
            name='visualization_node',
            output='screen'
        ),
        
        # Robot State Publisher
        Node(
            package='agrirover_manipulation',
            executable='robot_state_publisher_node',
            name='robot_state_publisher_node',
            output='screen'
        ),
        
        # Robot State Publisher (built-in)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_xml  # Load from file or parameter
            }],
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/pranav/agrirover_ws/src/agrirover_manipulation/rviz/agrirover_visualization.rviz'],
            output='screen'
        ),
    ])
