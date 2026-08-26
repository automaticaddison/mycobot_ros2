#!/bin/bash
set -e

# Set ROS 2 distribution as a variable
ROS_DISTRO="lyrical"

# Source ROS 2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Install system dependencies for MongoDB and PCL
apt-get update && apt-get install -y \
    gnupg \
    curl \
    libpcap-dev

# MongoDB and warehouse_ros_mongo have been dropped on Lyrical.
#
# warehouse_ros_mongo needs libmongoclient-dev, the old MongoDB C++ driver,
# and that package no longer exists on Ubuntu 26.04. MongoDB also has no
# "resolute" apt suite yet. Nothing in this repository ever used the MoveIt
# warehouse, so the whole MongoDB step is gone rather than worked around.
# warehouse_ros itself still comes in through rosdep as a MoveIt dependency.

# Navigate to the workspace
cd /root/ros2_ws/src

# MoveIt Task Constructor is a Debian package on Lyrical, so there is no need
# to clone and patch it from source anymore. Installed below with apt.

# Navigate back to the workspace root
cd /root/ros2_ws

# Install ROS2 dependencies for all packages
echo "Installing ROS 2 dependencies..."
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Install MoveIt Task Constructor from apt.
# On Jazzy this had to be cloned from source and patched by hand. Those patches
# targeted moveit::core::JumpThreshold, which no longer exists, so they are gone.
echo "Installing MoveIt Task Constructor..."
apt-get install -y \
    ros-$ROS_DISTRO-moveit-task-constructor-core \
    ros-$ROS_DISTRO-moveit-task-constructor-msgs \
    ros-$ROS_DISTRO-moveit-task-constructor-visualization

cd /root/ros2_ws

# Fix PCL warning - this needs to come after rosdep install
echo "Fixing PCL warnings..."
find /usr/include/pcl* -path "*/sample_consensus/impl/sac_model_plane.hpp" -exec sed -i 's/^\(\s*\)PCL_ERROR ("\[pcl::SampleConsensusModelPlane::isSampleGood\] Sample points too similar or collinear!\\n");/\1\/\/ PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Sample points too similar or collinear!\\n");/' {} \;

# Build the packages
echo "Building packages..."
# First build without the problematic package
colcon build --packages-skip mycobot_mtc_pick_place_demo
source install/setup.bash

# Then build the problematic package with warning suppression
colcon build --packages-select mycobot_mtc_pick_place_demo --cmake-args -Wno-dev
source install/setup.bash

# Final build of everything
colcon build
source install/setup.bash

echo "Workspace setup completed!"
