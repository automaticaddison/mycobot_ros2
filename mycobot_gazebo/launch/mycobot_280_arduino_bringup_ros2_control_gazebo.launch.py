# Author: Addison Sears-Collins
# Date: July 30, 2024
# Description: Launch a robotic arm in Gazebo 
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

  # Constants for paths to different files and folders
  package_name_description = 'mycobot_description'
  package_name_gazebo = 'mycobot_gazebo'

  default_robot_name = 'mycobot_280'
  gazebo_launch_file_path = 'launch'
  gazebo_models_path = 'models'
  rviz_config_file_path = 'rviz/mycobot_280_arduino_view_description.rviz'
  urdf_file_path = 'urdf/ros2_control/gazebo/mycobot_280.urdf.xacro'
  world_file_path = 'worlds/empty.world' # e.g. 'world/empty.world', 'world/house.world'

  # Set the path to different files and folders.  
  pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)

  default_rviz_config_path = os.path.join(pkg_share_description, rviz_config_file_path)  
  default_urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file_path)
  gazebo_launch_file_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)   
  gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
  world_path = os.path.join(pkg_share_gazebo, world_file_path)
  
  # Launch configuration variables specific to simulation
  robot_name = LaunchConfiguration('robot_name')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  world = LaunchConfiguration('world')
  
  # Set the default pose
  x = LaunchConfiguration('x')
  y = LaunchConfiguration('y')
  z = LaunchConfiguration('z')
  roll = LaunchConfiguration('roll')
  pitch = LaunchConfiguration('pitch')
  yaw = LaunchConfiguration('yaw')
  
  # Declare the launch arguments  
  declare_robot_name_cmd = DeclareLaunchArgument(
    name='robot_name',
    default_value=default_robot_name,
    description='The name for the robot')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

  declare_x_cmd = DeclareLaunchArgument(
    name='x',
    default_value='0.0',
    description='x component of initial position, meters')

  declare_y_cmd = DeclareLaunchArgument(
    name='y',
    default_value='0.0',
    description='y component of initial position, meters')
    
  declare_z_cmd = DeclareLaunchArgument(
    name='z',
    default_value='0.05',
    description='z component of initial position, meters')
    
  declare_roll_cmd = DeclareLaunchArgument(
    name='roll',
    default_value='0.0',
    description='roll angle of initial orientation, radians')

  declare_pitch_cmd = DeclareLaunchArgument(
    name='pitch',
    default_value='0.0',
    description='pitch angle of initial orientation, radians')

  declare_yaw_cmd = DeclareLaunchArgument(
    name='yaw',
    default_value='0.0',
    description='yaw angle of initial orientation, radians')
    
  # Specify the actions

  set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    gazebo_models_path)

  # Start arm controller
  start_arm_controller_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'arm_controller'],
        output='screen')

  # Start Gazebo environment
  start_gazebo_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments=[('gz_args', [' -r -v 4 ', world])])

  # Start gripper controller
  start_gripper_controller_cmd =  ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
        'grip_controller'],
        output='screen')

  # Start gripper action controller
  start_gripper_action_controller_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'grip_action_controller'],
        output='screen')
  
  # Launch joint state broadcaster
  start_joint_state_broadcaster_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'joint_state_broadcaster'],
        output='screen')
    
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  doc = xacro.parse(open(default_urdf_model_path))
  xacro.process_doc(doc)
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': doc.toxml()}])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file],
    parameters=[{'use_sim_time': use_sim_time}])  
    
   
  # Spawn the robot
  start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-string', doc.toxml(),
      '-name', robot_name,
      '-allow_renaming', 'true',
      '-x', x,
      '-y', y,
      '-z', z,
      '-R', roll,
      '-P', pitch,
      '-Y', yaw
      ],
    output='screen')

  # Launch the joint state broadcaster after spawning the robot
  load_joint_state_broadcaster_cmd = RegisterEventHandler(
     event_handler=OnProcessExit(
     target_action=start_gazebo_ros_spawner_cmd ,
     on_exit=[start_joint_state_broadcaster_cmd],))

  # Launch the arm controller after launching the joint state broadcaster
  load_arm_controller_cmd = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=start_joint_state_broadcaster_cmd,
    on_exit=[start_arm_controller_cmd],))

  # Launch the gripper controller after launching the arm controller
  load_gripper_controller_cmd = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=start_arm_controller_cmd,
    on_exit=[start_gripper_controller_cmd],))
    
  # Launch the gripper action controller after launching the gripper controller
  load_gripper_action_controller_cmd = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=start_gripper_controller_cmd,
    on_exit=[start_gripper_action_controller_cmd],))    

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_world_cmd)

  ld.add_action(declare_x_cmd)
  ld.add_action(declare_y_cmd)
  ld.add_action(declare_z_cmd)
  ld.add_action(declare_roll_cmd)
  ld.add_action(declare_pitch_cmd)
  ld.add_action(declare_yaw_cmd)  

  # Add any actions
  ld.add_action(set_env_vars_resources)
  ld.add_action(start_gazebo_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  
  ld.add_action(start_gazebo_ros_spawner_cmd)

  ld.add_action(load_joint_state_broadcaster_cmd)
  ld.add_action(load_arm_controller_cmd) 
  ld.add_action(load_gripper_controller_cmd) 
  ld.add_action(load_gripper_action_controller_cmd)

  return ld



