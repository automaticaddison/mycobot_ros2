#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the robot.

This script creates a launch description that starts the necessary controllers
for operating the robotic arm and gripper in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Arm Controller: Controls the robot arm movements via /follow_joint_trajectory
    3. Gripper Action Controller: Controls gripper actions via /gripper_action

Launch Sequence:
    1. Joint State Broadcaster
    2. Arm Controller (starts after Joint State Broadcaster)
    3. Gripper Action Controller (starts after Arm Controller)

:author: Addison Sears-Collins
:date: August 26, 2026
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.

    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    robot_name = LaunchConfiguration('robot_name')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_280',
        description='Name of the robot')

    # The controllers read their settings from this file. We hand it to each
    # spawner directly, because the controllers do not pick it up on their own.
    controllers_file = os.path.join(
        str(Path.home()),
        'ros2_ws/install/mycobot_moveit_config/share/mycobot_moveit_config/config',
        'mycobot_280',
        'ros2_controllers.yaml')

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--param-file', controllers_file],
            output='screen')

    start_joint_state_broadcaster_cmd = spawner('joint_state_broadcaster')
    start_arm_controller_cmd = spawner('arm_controller')
    start_gripper_action_controller_cmd = spawner('gripper_action_controller')

    # Give Gazebo time to come up before we ask for the first controller
    delayed_start = TimerAction(
        period=10.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    # Launch the arm controller after the joint state broadcaster
    load_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_arm_controller_cmd]))

    # Launch the gripper action controller after the arm controller
    load_gripper_action_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_arm_controller_cmd,
            on_exit=[start_gripper_action_controller_cmd]))

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(delayed_start)
    ld.add_action(load_arm_controller_cmd)
    ld.add_action(load_gripper_action_controller_cmd)

    return ld
