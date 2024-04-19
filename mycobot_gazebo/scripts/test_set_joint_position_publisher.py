#! /usr/bin/env python3

"""
Description:
  ROS 2: Executes a sample trajectory for a robotic arm in Gazebo
-------
Publishing Topics (ROS 2):
  Desired goal positions for joints on a robotic arm
    /model/mycobot_280/joint/link1_to_link2/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/link2_to_link3/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/link3_to_link4/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/link4_to_link5/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/link5_to_link6/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/link6_to_link6flange/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/gripper_controller/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/gripper_base_to_gripper_left2/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/gripper_left3_to_gripper_left1/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/gripper_base_to_gripper_right3/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/gripper_base_to_gripper_right2/cmd_pos - std_msgs/Float64
    /model/mycobot_280/joint/gripper_right3_to_gripper_right1/cmd_pos - std_msgs/Float64
-------
Author: Addison Sears-Collins
Date: April 18, 2024
"""
import numpy as np
import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Float64  # Import the Float64 message


# Define constants
names_of_joints = [ 'link1_to_link2', 
                    'link2_to_link3', 
                    'link3_to_link4', 
                    'link4_to_link5', 
                    'link5_to_link6', 
                    'link6_to_link6flange', 
                    'gripper_controller',
                    'gripper_base_to_gripper_left2',
                    'gripper_left3_to_gripper_left1',
                    'gripper_base_to_gripper_right3',
                    'gripper_base_to_gripper_right2',
                    'gripper_right3_to_gripper_right1']

class BasicJointPositionPublisher(Node):
    """This class executes a sample trajectory for a robotic arm
    
    """      
    def __init__(self):
        """ Constructor.
      
        """
        # Initialize the class using the constructor
        super().__init__('basic_joint_position_publisher')    
 
        # Create a publisher for each joint
        self.position_publishers = [
            self.create_publisher(Float64, f'/model/mycobot_280/joint/{name}/cmd_pos', 1)
            for name in names_of_joints
        ]
        self.timer_period = 0.05 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Starting position and goal position for the robotic arm joints
        self.start_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.end_position = [-1.345, -1.23, 0.264, -0.296, 0.389, -1.5, -0.7, -0.7, 0.7, 0.7, 0.7, -0.7]   

        # Number of steps to interpolate between positions
        self.num_steps = 50 
        
        # Set the desired goal poses for the robotic arm.
        self.positions = self.generate_positions(self.start_position, self.end_position)  
    
        # Keep track of the current trajectory we are executing
        self.index = 0

        # Indicate the direction of movement in the list of goal positions.
        self.forward = True

    def generate_positions(self, start_position, end_position):
        """
        Generates positions along a path from start to end positions.
        
        Args:
            start_position (list): The starting position of the robotic arm.
            end_position (list): The ending position of the robotic arm.
        
        Returns:
            list: A complete list of positions including all intermediate steps.
        """
        # Example path including start and end, could be expanded to more waypoints
        path_positions = [start_position, end_position]  
        all_positions = []
        for i in range(len(path_positions) - 1):
            interpolated = self.interpolate_positions(path_positions[i], path_positions[i + 1])
            all_positions.extend(interpolated[:-1])  # Exclude the last to avoid duplicates
        all_positions.append(path_positions[-1])  # Ensure the last position is included
        return all_positions

    def interpolate_positions(self, start, end):
        """
        Linearly interpolates between two positions.
        
        Args:
            start (list): The starting position for interpolation.
            end (list): The ending position for interpolation.
        
        Returns:
            list: A list of positions including the start, interpolated, and end positions.
        """
        interpolated_positions = [start]  # Initialize with the start position
        step_vector = (np.array(end) - np.array(start)) / (self.num_steps + 1)  # Calculate step vector
        for step in range(1, self.num_steps + 1):
            interpolated_position = np.array(start) + step * step_vector  # Compute each interpolated position
            interpolated_positions.append(interpolated_position.tolist())  # Append to the list
        interpolated_positions.append(end)  # Append the end position
        return interpolated_positions

    def timer_callback(self):
        """Set the goal pose for the robotic arm.
    
        """
        # Publish the current position for each joint
        for pub, pos in zip(self.position_publishers, self.positions[self.index]):
            msg = Float64()
            msg.data = pos
            pub.publish(msg)

        # Update the trajectory index
        if self.forward:
            if self.index < len(self.positions) - 1:
                self.index =  self.index + 1
            else:
                self.forward = False
        else:
            if self.index > 0:
                self.index = self.index - 1
            else:
                self.forward = True
    
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    basic_joint_position_publisher = BasicJointPositionPublisher()
  
    # Spin the node so the callback function is called.
    rclpy.spin(basic_joint_position_publisher)
    
    # Destroy the node
    basic_joint_position_publisher.destroy_node()
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
