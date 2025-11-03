"""
Improved Launch File for Agricultural Robot Arm Manipulation (FIXED)
- Pin 5 removed: 4 servos on pins 3, 6, 9, 10
- Updated home position: 4 angles instead of 5
- Camera attached to gripper (dynamic transform from URDF)
- Manual goal pose input with xterm visualization
- RViz marker visualization
- Improved node sequencing and error handling
- Configurable parameters
"""


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
    Launch configuration for agricultural robot arm with manual goal pose input.
    
    Key improvements:
    - Removed pin 5 servo (no hardware)
    - 4 servos on pins: 3, 6, 9, 10
    - Updated home position to 4 angles
    - Removed static TF publisher (camera is attached to gripper via URDF)
    - Better parameter organization
    - Improved logging and startup messaging
    - Conditional node launching
    - Workspace bounds aligned with actual robot reach
    """
    
    # ============================================================
    # DECLARE LAUNCH ARGUMENTS
    # ============================================================
    
    # Hardware Configuration
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor controller (e.g., /dev/ttyUSB0 or /dev/ttyACM0)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Serial communication baud rate'
    )
    
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Enable actual hardware control (motor serial communication)'
    )
    
    # Visualization Configuration
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
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
    
    # Robot Configuration
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
    
    # FIXED: 4 angles instead of 5
    home_position_arg = DeclareLaunchArgument(
        'home_position',
        default_value='[1.57, 0.0, 3.14, 3.14]',
        description='Home position joint angles in radians [joint1, joint2, joint3, joint4] - 4 servos'
    )
    
    # Movement Parameters
    interpolation_steps_arg = DeclareLaunchArgument(
        'interpolation_steps',
        default_value='20',
        description='Number of interpolation steps for smooth trajectory'
    )
    
    movement_duration_arg = DeclareLaunchArgument(
        'movement_duration',
        default_value='2.0',
        description='Duration of smooth movement in seconds'
    )
    
    use_easing_arg = DeclareLaunchArgument(
        'use_easing',
        default_value='true',
        description='Enable cubic ease-in-out function for smooth motion'
    )
    
    # Frame Configuration
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame (fixed reference)'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_head_link',
        description='Camera frame (attached to gripper)'
    )
    
    # Manual Input Configuration
    input_frame_arg = DeclareLaunchArgument(
        'input_frame',
        default_value='base_link',
        description='Reference frame for manual goal pose input (base_link or camera_head_link)'
    )
    
    # IK Solver Configuration
    workspace_x_min_arg = DeclareLaunchArgument(
        'workspace_x_min',
        default_value='-0.5',
        description='Minimum X reach of workspace (meters)'
    )
    
    workspace_x_max_arg = DeclareLaunchArgument(
        'workspace_x_max',
        default_value='0.5',
        description='Maximum X reach of workspace (meters)'
    )
    
    workspace_y_min_arg = DeclareLaunchArgument(
        'workspace_y_min',
        default_value='-0.5',
        description='Minimum Y reach of workspace (meters)'
    )
    
    workspace_y_max_arg = DeclareLaunchArgument(
        'workspace_y_max',
        default_value='0.5',
        description='Maximum Y reach of workspace (meters)'
    )
    
    workspace_z_min_arg = DeclareLaunchArgument(
        'workspace_z_min',
        default_value='0.0',
        description='Minimum Z reach of workspace (meters)'
    )
    
    workspace_z_max_arg = DeclareLaunchArgument(
        'workspace_z_max',
        default_value='0.8',
        description='Maximum Z reach of workspace (meters)'
    )
    
    # ============================================================
    # LOAD ROBOT DESCRIPTION FROM URDF
    # ============================================================
    
    try:
        robot_urdf_path = os.path.join(
            get_package_share_directory('agrirover_description'),
            'urdf', 'mech', 'arm_combined.urdf'
        )
        with open(robot_urdf_path, 'r') as f:
            robot_description_xml = f.read()
    except Exception as e:
        print(f"✗ Error loading URDF: {e}")
        robot_description_xml = ""
    
    # ============================================================
    # NODE DEFINITIONS
    # ============================================================
    
    # 1. Custom Robot State Publisher
    # Publishes joint states from servo commands
    # FIXED: Updated for 4 servos
    robot_state_publisher_node = Node(
        package='agrirover_manipulation',
        executable='robot_state_publisher_node',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[{
            'home_position': LaunchConfiguration('home_position'),
            'publish_rate': 50.0,
        }],
        emulate_tty=True,
        respawn=False,
        respawn_delay=2.0,
    )
    
    # 2. Built-in Robot State Publisher
    # Publishes TF transforms from joint states (uses URDF)
    builtin_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'publish_frequency': 50.0,
        }],
        emulate_tty=True,
    )
    
    # 3. IK Solver Node (Delayed start - wait for TF tree)
    # Converts goal poses to joint angles
    ik_solver_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='agrirover_manipulation',
                executable='ik_solver',
                name='ik_solver_node',
                output='screen',
                parameters=[{
                    'home_position': LaunchConfiguration('home_position'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'camera_frame': LaunchConfiguration('camera_frame'),
                    'tf_timeout': 2.0,
                    'ik_timeout': 1.0,
                    'max_ik_iterations': 100,
                    'position_tolerance': 0.01,
                    'num_ik_solutions': 5,
                    'manipulability_threshold': 0.01,
                    # Workspace bounds
                    'workspace_x_min': LaunchConfiguration('workspace_x_min'),
                    'workspace_x_max': LaunchConfiguration('workspace_x_max'),
                    'workspace_y_min': LaunchConfiguration('workspace_y_min'),
                    'workspace_y_max': LaunchConfiguration('workspace_y_max'),
                    'workspace_z_min': LaunchConfiguration('workspace_z_min'),
                    'workspace_z_max': LaunchConfiguration('workspace_z_max'),
                }],
                emulate_tty=True,
                respawn=True,
                respawn_delay=3.0,
            )
        ]
    )
    
    # 4. Serial Motor Commander Node
    # Sends joint commands to hardware or simulation
    # FIXED: Updated for 4 servos
    serial_motor_commander = Node(
        package='agrirover_manipulation',
        executable='serial_motor_commander',
        name='serial_motor_commander',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'enable_hardware': LaunchConfiguration('use_hardware'),
            'interpolation_steps': LaunchConfiguration('interpolation_steps'),
            'movement_duration': LaunchConfiguration('movement_duration'),
            'use_easing': LaunchConfiguration('use_easing'),
            'connection_retry_interval': 5.0,
            'max_connection_attempts': -1,  # Infinite retries
        }],
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
    )
    
    # 5. Manual Goal Pose Publisher (with xterm)
    # Interactive terminal for manual goal pose input
    # Publishes PoseStamped messages and MarkerArray for visualization
    manual_goal_pose_node = Node(
        package='agrirover_manipulation',
        executable='goal_pose_input',
        name='goal_pose_input',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('input_frame'),
            'marker_lifetime': 0.0,  # Persistent markers
            'history_size': 20,
        }],
        emulate_tty=True,
        prefix='xterm -hold -e',  # Opens in xterm, keeps window after exit
    )
    
    # 6. RViz2 Visualization (Delayed start)
    # 3D visualization of robot, goal poses, and markers
    rviz_node = TimerAction(
        period=3.0,
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
        # Launch Arguments
        # Hardware
        serial_port_arg,
        baud_rate_arg,
        use_hardware_arg,
        
        # Visualization
        use_rviz_arg,
        rviz_config_arg,
        
        # Robot Configuration
        urdf_file_arg,
        home_position_arg,
        
        # Movement Parameters
        interpolation_steps_arg,
        movement_duration_arg,
        use_easing_arg,
        
        # Frames
        base_frame_arg,
        camera_frame_arg,
        input_frame_arg,
        
        # Workspace Bounds
        workspace_x_min_arg,
        workspace_x_max_arg,
        workspace_y_min_arg,
        workspace_y_max_arg,
        workspace_z_min_arg,
        workspace_z_max_arg,
        
        # Startup Logging
        LogInfo(msg='═══════════════════════════════════════'),
        LogInfo(msg='  Agricultural Robot Arm System'),
        LogInfo(msg='═══════════════════════════════════════'),
        LogInfo(msg='Servos: 4 (pins 3, 6, 9, 10) - Pin 5 removed'),
        LogInfo(msg='Camera: Attached to gripper (dynamic transform)'),
        LogInfo(msg='Input: Manual goal poses via xterm'),
        LogInfo(msg='Visualization: RViz markers + robot model'),
        LogInfo(msg='═══════════════════════════════════════'),
        
        # Core Nodes (Start Immediately)
        robot_state_publisher_node,
        builtin_robot_state_publisher,
        # NOTE: Static TF publisher REMOVED
        # Camera transform is now dynamic from URDF
        
        # Delayed Nodes (Wait for TF tree)
        ik_solver_node,
        serial_motor_commander,
        manual_goal_pose_node,
        rviz_node,
    ])