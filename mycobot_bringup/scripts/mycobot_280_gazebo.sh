#!/bin/bash
# Single script to launch the myCobot with Gazebo and ROS 2 Controllers

cleanup() {
    echo "Cleaning up..."
    echo "Restarting ROS 2 daemon..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2-related processes..."
    kill 0
    exit
}

# Set up cleanup trap
trap 'cleanup' SIGINT

echo "Launching Gazebo simulation..."
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py




