import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        
        # Log the start of the fruit picking system
        LogInfo(msg="Launching the Fruit Picking System"),
        
        # Launch the cmd_vel to serial node
        Node(
            package='motor_commander',
            executable='cmd_serial_node',
            name='cmd_serial_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # Launch the goal sender node
        Node(
            package='motor_commander',
            executable='goal_sender_node',
            name='goal_sender_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # Launch the arm control node (e.g., task_node for servo control)
        Node(
            package='motor_commander',
            executable='task_node',
            name='arm_control_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # Launch the pick-and-place sequence node (from pymoveit2)
        Node(
            package='pymoveit2',
            executable='pick_and_place.py',
            name='pick_and_place_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
