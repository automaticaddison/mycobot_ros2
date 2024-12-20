#!/usr/bin/env python3

"""
Launch the get planning scene server.

This script launches the GetPlanningSceneServer node, which is responsible for managing
the planning scene in MoveIt. It sets up the necessary configuration and parameters,
including simulation time settings.

Launch Arguments:
    use_sim_time (bool): Use simulation clock if true, system clock if false

Configuration:
    The node uses configuration from 'config/get_planning_scene_server.yaml'
    in the mycobot_mtc_pick_place_demo package.

:author: Addison Sears-Collins
:date: December 19, 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the GetPlanningSceneServer.

    This function sets up the necessary paths, parameters, and node configurations
    for launching the GetPlanningSceneServer node.

    Returns:
        LaunchDescription: A complete launch description for the planning scene server
    """
    # Constants for paths to different files and folders
    package_name_mtc = 'mycobot_mtc_pick_place_demo'

    # Set the path to different files and folders
    pkg_share_mtc = FindPackageShare(package=package_name_mtc).find(package_name_mtc)

    # Paths for various configuration files
    get_planning_scene_server_file_path = 'config/get_planning_scene_server.yaml'

    # Set the full paths
    get_planning_scene_server_file_path = os.path.join(
        pkg_share_mtc, get_planning_scene_server_file_path)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Start the GetPlanningSceneServer node
    start_get_planning_scene_server_cmd = Node(
        package=package_name_mtc,
        executable="get_planning_scene_server",
        output="screen",
        parameters=[
            get_planning_scene_server_file_path,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_get_planning_scene_server_cmd)

    return ld
