#!/bin/bash
# Single script to launch point cloud viewing

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
#ros2 launch hello_mtc_with_perception point_cloud_viewer.launch.py file_name:=/home/ubuntu/Downloads/4_convertToPCL_debug_cloud.pcd
#ros2 launch hello_mtc_with_perception point_cloud_viewer.launch.py file_name:=/home/ubuntu/Downloads/5_support_plane_debug_cloud.pcd
ros2 launch hello_mtc_with_perception point_cloud_viewer.launch.py file_name:=/home/ubuntu/Downloads/5_objects_cloud_debug_cloud.pcd
