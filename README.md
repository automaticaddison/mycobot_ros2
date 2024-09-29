# mycobot_ros2 #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/jammy)
![ROS_2](https://img.shields.io/ros/v/iron/rclcpp)

## Overview
This repository contains ROS 2 packages for simulating and controlling the myCobot robotic arm using ROS 2 Control and MoveIt 2. It provides support for Gazebo simulation and visualization in RViz. Gazebo simulation also includes simulated 3D point cloud data from the depth camera (RGBD) sensor plugin for vision.

![Gazebo Pick and Place Task Simulation](https://automaticaddison.com/wp-content/uploads/2024/09/mtc_gazebo-moveit-pick-place.gif)

![Pick and Place with Perception](https://automaticaddison.com/wp-content/uploads/2024/09/pick-and-place-with-perception-moveit2-800px.gif)

## Features
- Gazebo simulation of the myCobot robotic arm
- RViz visualization for robot state and motion planning
- MoveIt 2 integration for motion planning and control
- Pick and place task implementation using the MoveIt Task Constructor (MTC)
- 3D perception and object segmentation using point cloud data
- Automatic planning scene generation from perceived objects
- Support for various primitive shapes (cylinders, boxes) in object detection
- Integration with tf2 for coordinate transformations
- Custom service for retrieving planning scene information
- Advanced object detection algorithms:
  - RANSAC (Random Sample Consensus) for robust model fitting
  - Hough transform for shape recognition
- CPU-compatible implementation, no GPU required. 
- Real-time perception and planning capabilities for responsive robot operation

![Setup Planning Scene](https://automaticaddison.com/wp-content/uploads/2024/09/setup-planning-scene-800px.gif)

## Getting Started
For a complete step-by-step walkthrough on how to build this repository from scratch, start with this tutorial:
[How to Model a Robotic Arm with a URDF File (ROS 2)](https://automaticaddison.com/how-to-model-a-robotic-arm-with-a-urdf-file-ros-2/)

This guide will take you through the entire process of setting up and understanding the mycobot_ros2 project.

![3D Point Cloud RViz](https://automaticaddison.com/wp-content/uploads/2024/09/3d-point-cloud-rviz.jpg)

![mycobot280_rviz](./mycobot_description/urdf/mycobot280_rviz.png)
