# Author: Addison Sears-Collins
# Date: September 11, 2024
# Description: Launch the get planning scene server.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
 

def generate_launch_description():
    # Constants for paths to different files and folders
    package_name_mtc = 'hello_mtc_with_perception'

    # Set the path to different files and folders
    pkg_share_mtc = FindPackageShare(package=package_name_mtc).find(package_name_mtc)

    # Paths for various configuration files
    get_planning_scene_server_file_path = 'config/get_planning_scene_server.yaml'    

    # Set the full paths
    get_planning_scene_server_file_path = os.path.join(pkg_share_mtc, get_planning_scene_server_file_path)

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
        parameters=[get_planning_scene_server_file_path],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_get_planning_scene_server_cmd)

    return ld
    
