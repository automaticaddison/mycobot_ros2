#!/usr/bin/env python3
"""
Launch MoveIt 2 for the myCobot robotic arm.

This script creates a ROS 2 launch file that starts the necessary nodes and services
for controlling a myCobot robotic arm using MoveIt 2. It loads configuration files,
starts the move_group node, and optionally launches RViz for visualization.

:author: Addison Sears-Collins
:date: December 13, 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.contexts import LaunchContext


def generate_launch_description():
    """
    Generate a launch description for MoveIt 2 with myCobot robot.

    This function sets up the necessary configuration and nodes to launch MoveIt 2
    for controlling a myCobot robotic arm. It includes setting up paths to config files,
    declaring launch arguments, configuring the move_group node, and optionally starting RViz.

    The configuration files are organized in a two-level structure:
    - Robot-specific configs in config/<robot_name>/
    - Planning pipeline configs directly in config/

    Returns:
        LaunchDescription: A complete launch description for the MoveIt 2 system
    """
    # Launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_package = LaunchConfiguration('rviz_config_package')

    # Constants for paths to different files and folders
    package_name_moveit_config = 'mycobot_moveit_config'

    # Set the path to different files and folders
    pkg_share_moveit_config = FindPackageShare(
        package=package_name_moveit_config).find(package_name_moveit_config)

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_280',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='move_group.rviz',
        description='RViz configuration file')

    declare_rviz_config_package_cmd = DeclareLaunchArgument(
        name='rviz_config_package',
        default_value=package_name_moveit_config,
        description='Package containing the RViz configuration file')

    # Define robot-specific config directory
    robot_config_dir = os.path.join('config', robot_name)

    # Paths for robot-specific configuration files
    initial_positions_file_path = os.path.join(robot_config_dir, 'initial_positions.yaml')
    joint_limits_file_path = os.path.join(robot_config_dir, 'joint_limits.yaml')
    kinematics_file_path = os.path.join(robot_config_dir, 'kinematics.yaml')
    moveit_controllers_file_path = os.path.join(robot_config_dir, 'moveit_controllers.yaml')
    srdf_file_path = os.path.join(robot_config_dir, f'{robot_name}.srdf')
    pilz_cartesian_limits_file_path = os.path.join(robot_config_dir, 'pilz_cartesian_limits.yaml')

    # Set the full paths for robot-specific configs
    initial_positions_file_path = os.path.join(pkg_share_moveit_config, initial_positions_file_path)
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, joint_limits_file_path)
    kinematics_file_path = os.path.join(pkg_share_moveit_config, kinematics_file_path)
    moveit_controllers_file_path = os.path.join(
        pkg_share_moveit_config, moveit_controllers_file_path)
    srdf_model_path = os.path.join(pkg_share_moveit_config, srdf_file_path)
    pilz_cartesian_limits_file_path = os.path.join(
        pkg_share_moveit_config, pilz_cartesian_limits_file_path)

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=package_name_moveit_config)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        # Planning pipeline configs are automatically found in config/ directory
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    # Enable the execution of tasks in the MoveIt Task Constructor (MTC).
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            {'start_state': {'content': initial_positions_file_path}},
            move_group_capabilities,
        ],
    )

    # RViz
    start_rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            [FindPackageShare(rviz_config_package), "/rviz/", rviz_config_file]
        ],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time}
        ],
    )

    exit_event_handler = RegisterEventHandler(
        condition=IfCondition(use_rviz),
        event_handler=OnProcessExit(
            target_action=start_rviz_node_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_config_package_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add any actions
    ld.add_action(start_move_group_node_cmd)
    ld.add_action(start_rviz_node_cmd)

    # Clean shutdown of RViz
    ld.add_action(exit_event_handler)

    return ld
