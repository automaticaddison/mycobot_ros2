#! /usr/bin/env python3

"""
Description:
  ROS 2: Executes a sample trajectory for a robotic arm in Gazebo
-------
Publishing Topics (ROS 2):
  Desired goal pose of the robotic arm
    /arm_controller/joint_trajectory - trajectory_msgs/JointTrajectory
  
  Desired goal pose of the gripper
    /grip_controller/joint_trajectory - trajectory_msgs/JointTrajectory
-------
Author: Addison Sears-Collins
Date: April 29, 2024
"""

import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header  # Import the Header message

# Define constants
arm_joints = [ 'link1_to_link2', 
               'link2_to_link3', 
               'link3_to_link4', 
               'link4_to_link5', 
               'link5_to_link6', 
               'link6_to_link6flange']

gripper_joints = ['gripper_controller']

class ExampleJointTrajectoryPublisherPy(Node):
    """This class executes a sample trajectory for a robotic arm
    
    """      
    def __init__(self):
        """ Constructor.
      
        """
        # Initialize the class using the constructor
        super().__init__('example_joint_trajectory_publisher_py')    
 
        # Create the publisher of the desired arm and gripper goal poses
        self.arm_pose_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.gripper_pose_publisher = self.create_publisher(JointTrajectory, '/grip_controller/joint_trajectory', 1)

        self.timer_period = 5.0  # seconds 5.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.frame_id = "base_link"
       
        # Desired time from the trajectory start to arrive at the trajectory point.
        # Needs to be less than or equal to the self.timer_period above to allow
        # the robotic arm to smoothly transition between points.
        self.duration_sec = 2 
        self.duration_nanosec = 0.5 * 1e9 # (seconds * 1e9)

        # Set the desired goal poses for the robotic arm.
        self.arm_positions = []
        self.arm_positions.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Home location
        self.arm_positions.append([-1.345, -1.23, 0.264, -0.296, 0.389, -1.5]) # Goal location
        self.arm_positions.append([-1.345, -1.23, 0.264, -0.296, 0.389, -1.5]) 
        self.arm_positions.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Home location

        self.gripper_positions = []
        self.gripper_positions.append([0.0]) # Open gripper
        self.gripper_positions.append([0.0])
        self.gripper_positions.append([-0.70]) # Close gripper
        self.gripper_positions.append([-0.70]) 

        # Keep track of the current trajectory we are executing
        self.index = 0

    def timer_callback(self):
        """Set the goal pose for the robotic arm.
    
        """
        # Create new JointTrajectory messages
        msg_arm = JointTrajectory()
        msg_arm.header = Header()  
        msg_arm.header.frame_id = self.frame_id  
        msg_arm.joint_names = arm_joints

        msg_gripper = JointTrajectory()
        msg_gripper.header = Header()  
        msg_gripper.header.frame_id = self.frame_id  
        msg_gripper.joint_names = gripper_joints

        # Create JointTrajectoryPoints
        point_arm = JointTrajectoryPoint()
        point_arm.positions = self.arm_positions[self.index]
        point_arm.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))  # Time to next position
        msg_arm.points.append(point_arm)
        self.arm_pose_publisher.publish(msg_arm)

        point_gripper = JointTrajectoryPoint()
        point_gripper.positions = self.gripper_positions[self.index]
        point_gripper.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))  
        msg_gripper.points.append(point_gripper)
        self.gripper_pose_publisher.publish(msg_gripper)

        # Reset the index
        if self.index == len(self.arm_positions) - 1:
            self.index = 0
        else:
            self.index = self.index + 1
    
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    example_joint_trajectory_publisher_py = ExampleJointTrajectoryPublisherPy()
  
    # Spin the node so the callback function is called.
    rclpy.spin(example_joint_trajectory_publisher_py)
    
    # Destroy the node
    example_joint_trajectory_publisher_py.destroy_node()
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
