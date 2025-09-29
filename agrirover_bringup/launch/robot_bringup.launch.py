# launch/robot_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='moveit.rviz',
        description='RViz configuration file'
    )
    
    robot_urdf_path = PathJoinSubstitution([
        FindPackageShare("agrirover_description"),
        "urdf/robots/arm.urdf.xacro"
        ])
    
    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("", package_name="agrirover_moveit_config")
        .robot_description(file_path="config/agrirover/arm_mech.urdf")
        .robot_description_semantic(file_path="config/agrirover/arm.srdf")
        .trajectory_execution(file_path="config/agrirover/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    # RViz Node
    rviz_config = PathJoinSubstitution([
        FindPackageShare("agrirover_moveit_config"),
        "rviz", 
        LaunchConfiguration("rviz_config")
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    
    # ROS2 Control Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, moveit_config.to_dict()],
        output="screen",
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )
    
    # Arduino Serial Communication Node
    arduino_serial_node = Node(
        package="rosserial_python",
        executable="serial_node.py",
        name="arduino_serial_node",
        parameters=[{
            "port": "/dev/ttyUSB0",
            "baud": 115200
        }],
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Joint Trajectory Controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Static Transform Publisher (if needed)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )
    
    return LaunchDescription([
        use_sim_arg,
        rviz_config_arg,
        static_tf_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        ros2_control_node,
        arduino_serial_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
