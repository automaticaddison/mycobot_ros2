# Author: Addison Sears-Collins
# Date: July 31, 2024
# Description: Launch MoveIt 2 for the myCobot robotic arm

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():

    # Constants for paths to different files and folders
    package_name_gazebo = 'mycobot_gazebo'
    package_name_moveit_config = 'mycobot_moveit_config_manual_setup'

    # Set the path to different files and folders
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)

    # Paths for various configuration files
    urdf_file_path = 'urdf/ros2_control/gazebo/mycobot_280.urdf.xacro'
    srdf_file_path = 'config/mycobot_280.srdf'
    moveit_controllers_file_path = 'config/moveit_controllers.yaml'
    joint_limits_file_path = 'config/joint_limits.yaml'
    kinematics_file_path = 'config/kinematics.yaml'
    pilz_cartesian_limits_file_path = 'config/pilz_cartesian_limits.yaml'
    initial_positions_file_path = 'config/initial_positions.yaml'
    rviz_config_file_path = 'rviz/move_group.rviz'

    # Set the full paths
    urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file_path)
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, moveit_controllers_file_path)
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, joint_limits_file_path)
    kinematics_file_path = os.path.join(pkg_share_moveit_config, kinematics_file_path)
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, pilz_cartesian_limits_file_path)
    initial_positions_file_path = os.path.join(pkg_share_moveit_config, initial_positions_file_path)
    rviz_config_file = os.path.join(pkg_share_moveit_config, rviz_config_file_path)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_static_tf = LaunchConfiguration('use_static_tf')
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name=package_name_moveit_config)
        .robot_description(file_path=urdf_model_path)
        .robot_description_semantic(file_path=srdf_file_path)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            {'default_planning_pipeline': 'pilz_industrial_motion_planner'},
            {'start_state': {'content': initial_positions_file_path}}
        ],
    )

    # RViz
    start_rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add any actions
    ld.add_action(start_move_group_node_cmd)
    ld.add_action(start_rviz_node_cmd)

    return ld
