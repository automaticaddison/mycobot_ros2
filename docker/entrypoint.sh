#!/bin/bash

# Set ROS distribution
# This variable defines the ROS 2 distribution being used (e.g., "jazzy", "rolling", etc.)
ROS_DISTRO="jazzy"

# Creates required temp directory for GUI applications - needed by Linux desktop standards
# Set up XDG runtime directory
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Set base paths for convenience
# ROS_WS is the path to the ROS 2 workspace
ROS_WS="/root/ros2_ws"

# SHARED_ROS2 is the path to shared ROS 2 files
SHARED_ROS2="/root/shared/ros2"

# Create ros_domain_id.txt file with default value 0 if it doesn't exist
# This file stores the ROS_DOMAIN_ID, which is used for multi-robot communication
ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"

if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
    # If the file doesn't exist, create it with a default value of 0
    echo "0" > "$ROS_DOMAIN_ID_FILE"
    echo "Created $ROS_DOMAIN_ID_FILE with default value 0"
fi

# The ROS_DOMAIN_ID will need to be the same for computers that you want to communicate with each other.
# This value can take any number between [0,232]. It is useful if you have multiple robots in the same environment.
# Update .bashrc with ROS_DOMAIN_ID if it does not already exist
if ! grep -q "export ROS_DOMAIN_ID" /root/.bashrc; then
  # Read the ROS_DOMAIN_ID from the file
  ros_domain_id=$(cat "$ROS_DOMAIN_ID_FILE")
  # Add the export command to .bashrc if it's not already there
  echo "export ROS_DOMAIN_ID=$ros_domain_id" >> /root/.bashrc
fi

# Make the ROS Domain ID available to the current script
# This allows the current script to use the ROS_DOMAIN_ID
export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")

# Source ROS setup file
# This sets up the ROS 2 environment for the current shell session
source /opt/ros/$ROS_DISTRO/setup.bash

# Source the local workspace setup
source $ROS_WS/install/setup.bash

# Source the .bashrc file to ensure all environment variables are up to date
source /root/.bashrc

# Change to the ROS 2 workspace directory
cd $ROS_WS

# Build the ROS 2 workspace using colcon
colcon build

# Change back to the root directory
cd

# Source .bashrc again to ensure any changes made during the build are applied
source /root/.bashrc

# Any command you run after this script runs in the environment set up by the script.
exec "$@"