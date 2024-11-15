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

echo "Launching robot state publisher..."
ros2 launch mycobot_description robot_state_publisher.launch.py \
    jsp_gui:=false \
    use_rviz:=true \
    use_gazebo:=true &

sleep 3

echo "Launching Gazebo simulation..."
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py \
   world_file:=empty.world &

#sleep 5

#echo "Loading ROS 2 controllers..."
#ros2 launch mycobot_moveit_config load_ros2_controllers.launch.py
