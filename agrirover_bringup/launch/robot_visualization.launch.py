from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, 
    TimerAction, 
    LogInfo,
)
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import launch


def generate_launch_description():
    """
    Enhanced launch file with:
    - Proper startup sequencing
    - Configurable parameters via launch arguments
    - Better error handling
    - Conditional node launching
    - Camera-based goal pose publishing (no manual input)
    """
    
    # ============================================================
    # DECLARE LAUNCH ARGUMENTS
    # ============================================================
    
    # Hardware arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor controller'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Enable hardware (serial) communication'
    )
    
    # Visualization arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('agrirover_manipulation'),
            'rviz',
            'agrirover_visualization.rviz'
        ]),
        description='Path to RViz configuration file'
    )
    
    # Robot description arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agrirover_description'),
            'urdf',
            'mech',
            'arm_combined.urdf'
        ]),
        description='Path to robot URDF file'
    )
    
    # IK solver arguments
    home_position_arg = DeclareLaunchArgument(
        'home_position',
        default_value='[1.57, 1.57, 0.9, 0.8, 0.0]',
        description='Home position joint angles in radians'
    )
    
    # Movement parameters
    interpolation_steps_arg = DeclareLaunchArgument(
        'interpolation_steps',
        default_value='20',
        description='Number of interpolation steps for smooth movement'
    )
    
    movement_duration_arg = DeclareLaunchArgument(
        'movement_duration',
        default_value='2.0',
        description='Duration of movement in seconds'
    )
    
    use_easing_arg = DeclareLaunchArgument(
        'use_easing',
        default_value='true',
        description='Use easing function for smoother motion'
    )
    
    # Frame IDs
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame of the robot'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_head_link',
        description='Camera frame ID'
    )
    
    # Camera arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='webcam',
        description='Camera type: webcam or realsense'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID (for webcam)'
    )
    
    z_distance_arg = DeclareLaunchArgument(
        'z_distance',
        default_value='0.5',
        description='Depth distance for webcam (in meters)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/pranav/agrirover_ws/src/yolov8n_tomato.pt',
        description='Path to YOLO model file'
    )
    
    detection_timeout_arg = DeclareLaunchArgument(
        'detection_timeout',
        default_value='3.0',
        description='Timeout for detection before moving home (seconds)'
    )
    
    # ============================================================
    # LOAD ROBOT DESCRIPTION
    # ============================================================
    
    try:
        robot_urdf_path = os.path.join(
            get_package_share_directory('agrirover_description'),
            'urdf', 'mech', 'arm_combined.urdf'
        )
        with open(robot_urdf_path, 'r') as f:
            robot_description_xml = f.read()
    except Exception as e:
        print(f"Error loading URDF: {e}")
        robot_description_xml = ""
    
    # ============================================================
    # NODE DEFINITIONS
    # ============================================================
    
    # 1. Robot State Publisher (custom) - Publishes joint states
    robot_state_publisher_node = Node(
        package='agrirover_manipulation',
        executable='robot_state_publisher_node',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[{
            'home_position': LaunchConfiguration('home_position'),
        }],
        emulate_tty=True,
        respawn=False,
        respawn_delay=2.0,
    )
    
    # 2. Robot State Publisher (built-in) - Publishes TF transforms
    builtin_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'publish_frequency': 50.0,  # Hz
        }],
        emulate_tty=True,
    )
    
    # 4. IK Solver Node - Delayed start to wait for transforms
    ik_solver_node = TimerAction(
        period=2.0,  # Wait 2 seconds for TF tree to be ready
        actions=[
            Node(
                package='agrirover_manipulation',
                executable='ik_solver',
                name='ik_solver_node',
                output='screen',
                parameters=[{
                    'home_position': LaunchConfiguration('home_position'),
                    'tf_timeout': 2.0,
                    'ik_timeout': 1.0,
                    'max_ik_iterations': 100,
                    'position_tolerance': 0.01,
                    'num_ik_solutions': 5,
                    'manipulability_threshold': 0.01,
                    'base_frame': LaunchConfiguration('base_frame'),
                    'camera_frame': LaunchConfiguration('camera_frame'),
                }],
                emulate_tty=True,
                respawn=True,
                respawn_delay=3.0,
            )
        ]
    )
    
    # 5. Serial Motor Commander Node
    serial_motor_commander = Node(
        package='agrirover_manipulation',
        executable='serial_motor_commander',
        name='serial_motor_commander',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'interpolation_steps': LaunchConfiguration('interpolation_steps'),
            'movement_duration': LaunchConfiguration('movement_duration'),
            'use_easing': LaunchConfiguration('use_easing'),
            'enable_hardware': LaunchConfiguration('use_hardware'),
        }],
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
    )
    
    # 6. Camera Detection Node (NEW - replaces manual goal publisher)
    camera_node = Node(
        package='agrirover_manipulation',
        executable='camera_node',
        name='tomato_detection_camera_node',
        output='screen',
        parameters=[{
            'camera_type': LaunchConfiguration('camera_type'),
            'camera_id': LaunchConfiguration('camera_id'),
            'z_distance': LaunchConfiguration('z_distance'),
            'model_path': LaunchConfiguration('model_path'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
            'timer_period': 0.033,  # 30 Hz
        }],
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )
    
    # 7. RViz2 - Delayed start
    rviz_node = TimerAction(
        period=3.0,  # Wait for robot state publisher
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/home/pranav/agrirover_ws/src/agrirover_manipulation/rviz/agrirover_visualization.rviz'],
                output='screen',
                emulate_tty=True,
                condition=launch.conditions.IfCondition(
                    LaunchConfiguration('use_rviz')
                ),
            )
        ]
    )
    
    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================
    
    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baud_rate_arg,
        use_hardware_arg,
        use_rviz_arg,
        rviz_config_arg,
        urdf_file_arg,
        home_position_arg,
        interpolation_steps_arg,
        movement_duration_arg,
        use_easing_arg,
        base_frame_arg,
        camera_frame_arg,
        camera_type_arg,
        camera_id_arg,
        z_distance_arg,
        model_path_arg,
        detection_timeout_arg,
        
        # Core nodes (start immediately)
        LogInfo(msg='Starting Agricultural Robot Arm Manipulation System...'),
        robot_state_publisher_node,
        builtin_robot_state_publisher,
        
        # Delayed nodes (wait for TF tree)
        ik_solver_node,
        serial_motor_commander,
        camera_node,  # Camera provides goal poses automatically
        rviz_node,
    ])
