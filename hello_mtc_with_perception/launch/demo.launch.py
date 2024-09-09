# Author: Addison Sears-Collins
# Date: August 15, 2024
# Description: Launch MoveIt 2 for the myCobot robotic arm 

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
import xacro


def generate_launch_description():

    # Constants for paths to different files and folders
    package_name_gazebo = 'mycobot_gazebo'
    package_name_moveit_config = 'mycobot_moveit_config_manual_setup'
    package_name_mtc = 'hello_mtc_with_perception'

    # Set the path to different files and folders
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
    pkg_share_mtc = FindPackageShare(package=package_name_mtc).find(package_name_mtc)

    # Paths for various configuration files
    urdf_file_path = 'urdf/mycobot_280.urdf.xacro'
    srdf_file_path = 'config/mycobot_280.srdf'
    moveit_controllers_file_path = 'config/moveit_controllers.yaml'
    joint_limits_file_path = 'config/joint_limits.yaml'
    kinematics_file_path = 'config/kinematics.yaml'
    pilz_cartesian_limits_file_path = 'config/pilz_cartesian_limits.yaml'
    initial_positions_file_path = 'config/initial_positions.yaml'
    rviz_config_file_path = 'rviz/mtc.rviz'

    # Set the full paths
    urdf_model_path = os.path.join(pkg_share_mtc, urdf_file_path)
    srdf_model_path = os.path.join(pkg_share_moveit_config, srdf_file_path)
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, moveit_controllers_file_path)
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, joint_limits_file_path)
    kinematics_file_path = os.path.join(pkg_share_moveit_config, kinematics_file_path)
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, pilz_cartesian_limits_file_path)
    initial_positions_file_path = os.path.join(pkg_share_moveit_config, initial_positions_file_path)
    rviz_config_file = os.path.join(pkg_share_moveit_config, rviz_config_file_path)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

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
    # Typically, you would also have this line in here: .robot_description(file_path=urdf_model_path)
    # Another launch file is launching the robot description.
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name=package_name_moveit_config)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
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
        arguments=["-d", rviz_config_file],
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
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add any actions
    ld.add_action(start_move_group_node_cmd)
    ld.add_action(start_rviz_node_cmd)
    
    # Clean shutdown of RViz
    ld.add_action(exit_event_handler)

    return ld
    
