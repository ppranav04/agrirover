from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name_moveit_config = 'agrirover_moveit_config'
    robot_name_str = 'agrirover'

    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
    config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)

    # File paths for MoveIt configuration
    initial_positions = os.path.join(config_path, 'initial_positions.yaml')
    joint_limits = os.path.join(config_path, 'joint_limits.yaml')
    kinematics = os.path.join(config_path, 'kinematics.yaml')
    moveit_controllers = os.path.join(config_path, 'moveit_controllers.yaml')
    srdf = os.path.join(config_path, 'arm.srdf')
    pilz_cartesian_limits = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

    # These YAMLs need to exist and be valid for your robot
    from moveit_configs_utils import MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
        .trajectory_execution(file_path=moveit_controllers)
        .robot_description_semantic(file_path=srdf)
        .joint_limits(file_path=joint_limits)
        .robot_description_kinematics(file_path=kinematics)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits)
        .to_moveit_configs()
    )

    # Minimal move_group node
    start_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
            {'start_state': {'content': initial_positions}},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    return LaunchDescription([start_move_group_node])
