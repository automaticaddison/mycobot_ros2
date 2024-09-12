# Author: Addison Sears-Collins
# Date: September 12, 2024
# Description: ROS 2 launch file for viewing .pcd (point cloud data) files in RViz.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
    # Constants for paths to different files and folders
    package_name = 'hello_mtc_with_perception'

    # Set the path to different files and folders
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Paths for various configuration files
    rviz_config_file_path = 'rviz/point_cloud_viewer.rviz'

    # Set the full paths
    rviz_config_file = os.path.join(pkg_share, rviz_config_file_path)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    file_name = LaunchConfiguration('file_name')
    frame_id = LaunchConfiguration('frame_id')
    interval = LaunchConfiguration('interval')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_file_name_cmd = DeclareLaunchArgument(
        name='file_name',
        default_value='/home/ubuntu/Downloads/debug_cloud.pcd',
        description='Path to the PCD file')

    declare_frame_id_cmd = DeclareLaunchArgument(
        name='frame_id',
        default_value='base_link',
        description='Frame ID for the point cloud')

    declare_interval_cmd = DeclareLaunchArgument(
        name='interval',
        default_value='1.0',
        description='Interval for publishing the point cloud')

    # Start the PCL node
    start_pcl_node_cmd = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="pcd_to_pointcloud",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time},
            {'file_name': file_name},
            {'frame_id': frame_id},
            {'interval': interval},
        ],
    )

    # RViz
    start_rviz_node_cmd = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_file_name_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(declare_interval_cmd)

    # Add any actions
    ld.add_action(start_pcl_node_cmd)
    ld.add_action(start_rviz_node_cmd)

    return ld
    
