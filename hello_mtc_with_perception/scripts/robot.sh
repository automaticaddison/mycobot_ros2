#!/bin/bash
# Single script to launch the mycobot with Gazebo, RViz, and MoveIt 2

cleanup() {
    echo "Restarting ROS 2 daemon to cleanup before shutting down all processes..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2-related processes..."
    kill 0
    exit
}

trap 'cleanup' SIGINT;
ros2 launch hello_mtc_with_perception robot.launch.py use_rviz:=false &
sleep 10
ros2 launch hello_mtc_with_perception demo.launch.py &
sleep 10
ros2 launch hello_mtc_with_perception get_planning_scene_server.launch.py &
sleep 5
ros2 launch hello_mtc_with_perception run.launch.py
