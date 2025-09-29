from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include main launch file with hardware flag
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("agrirover_bringup"),
                "launch",
                "robot_bringup.launch.py"
            ])
        ]),
        launch_arguments={'use_sim': 'false'}.items(),
    )
    
    return LaunchDescription([main_launch])
