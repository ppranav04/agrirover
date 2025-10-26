from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


robot_urdf_path = os.path.join(
    get_package_share_directory('agrirover_description'),
    'urdf',
    'mech',
    'arm_mech.urdf'
)

with open(robot_urdf_path, 'r') as f:
    robot_description_xml = f.read()

def generate_launch_description():
    # Declare launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('agrirover_manipulation'),
            'rviz',
            'robot_control.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # Robot State Publisher (publishes TF transformations)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': False
        }]
    )
    
    # Joint State Publisher (publishes joint states)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 50,
            'source_list': ['joint_command']
        }]
    )

    # Optional: Interactive Marker Server for RViz control
    interactive_marker_node = Node(
        package='agrirover_manipulation',
        executable='interactive_marker_server',
        name='interactive_marker_server',
        output='screen'
    )
    
    # IK Solver Node
    ik_solver_node = Node(
        package='agrirover_manipulation',
        executable='ik_solver',
        name='ik_solver',
        output='screen',
        parameters=[{
            'solver_timeout': 0.1,
            'max_iterations': 100
        }],
        remappings=[
            ('/target_pose', '/ik_solver/target_pose'),
            ('/joint_angles', '/ik_solver/joint_angles')
        ]
    )
    
    # Motor Commander Node
    motor_commander_node = Node(
        package='agrirover_manipulation',
        executable='serial_motor_commander',
        name='motor_commander',
        output='screen',
        parameters=[{
            'control_rate': 100,
            'use_position_control': True
        }],
        remappings=[
            ('/joint_command', '/motor_commander/joint_command'),
            ('/joint_states', '/motor_commander/joint_states')
        ]
    )
    
    # RViz Node with Interactive Markers
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    

    
    return LaunchDescription([
        use_rviz,
        rviz_config_arg,
        robot_state_publisher,
        joint_state_publisher,
        interactive_marker_node,
        ik_solver_node,
        motor_commander_node,
        rviz_node,
    ])
