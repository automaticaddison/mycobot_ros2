# Author: Addison Sears-Collins
# Date: April 6, 2024
# Description: Launch a robotic arm in Gazebo
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  # Constants for paths to different files and folders
  package_name_description = 'mycobot_description'
  package_name_gazebo = 'mycobot_gazebo'

  #robot_name_in_model = 'mycobot_280'
  gazebo_models_path = 'models'
  gazebo_launch_file_path = 'launch'
  #rviz_config_file_path = 'mycobot_280_arduino_view_description.rviz'
  #urdf_file_path = 'urdf/mycobot_280_gazebo.urdf.xacro'
  world_file_path = 'worlds/empty.world' # Example: 'worlds/house.world', 'worlds/empty.world'

  # Set the path to different files and folders.  
  pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
  
  #default_urdf_model_path = os.path.join(package_name_gazebo, urdf_file_path)
  #default_rviz_config_path = os.path.join(pkg_share_description, rviz_config_file_path)

  gazebo_launch_file_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)   
  gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
  world_path = os.path.join(pkg_share_gazebo, world_file_path)
  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  #jsp_gui = LaunchConfiguration('jsp_gui')
  #rviz_config_file = LaunchConfiguration('rviz_config_file')
  #urdf_model = LaunchConfiguration('urdf_model')
  #use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  #use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  
  # Set the default pose
  #x = LaunchConfiguration('x')
  #y = LaunchConfiguration('y')
  #z = LaunchConfiguration('z')
  #roll = LaunchConfiguration('roll')
  #pitch = LaunchConfiguration('pitch')
  #yaw = LaunchConfiguration('yaw')
  
  # Declare the launch arguments  
  # declare_jsp_gui_cmd = DeclareLaunchArgument(
    # name='jsp_gui', 
    # default_value='true', 
    # choices=['true', 'false'],
    # description='Flag to enable joint_state_publisher_gui')
        
  # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    # name='rviz_config_file',
    # default_value=default_rviz_config_path,
    # description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Display the Gazebo GUI if False, otherwise run in headless mode')

  # declare_urdf_model_path_cmd = DeclareLaunchArgument(
    # name='urdf_model', 
    # default_value=default_urdf_model_path, 
    # description='Absolute path to robot urdf file')
    
  # declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    # name='use_robot_state_pub',
    # default_value='True',
    # description='Whether to start the robot state publisher')

  # declare_use_rviz_cmd = DeclareLaunchArgument(
    # name='use_rviz',
    # default_value='False',
    # description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start Gazebo')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

  # declare_x_cmd = DeclareLaunchArgument(
    # name='x',
    # default_value='0.0',
    # description='x component of initial position, meters')

  # declare_y_cmd = DeclareLaunchArgument(
    # name='y',
    # default_value='0.0',
    # description='y component of initial position, meters')
    
  # declare_z_cmd = DeclareLaunchArgument(
    # name='z',
    # default_value='0.0',
    # description='z component of initial position, meters')
    
  # declare_roll_cmd = DeclareLaunchArgument(
    # name='roll',
    # default_value='0.0',
    # description='roll angle of initial orientation, radians')

  # declare_pitch_cmd = DeclareLaunchArgument(
    # name='pitch',
    # default_value='0.0',
    # description='pitch angle of initial orientation, radians')

  # declare_yaw_cmd = DeclareLaunchArgument(
    # name='yaw',
    # default_value='0.0',
    # description='yaw angle of initial orientation, radians')
    
  # Specify the actions

  set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    gazebo_models_path)
  
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-g -v4 '}.items(),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
  # Publish the joint state values for the non-fixed joints in the URDF file.
  # start_joint_state_publisher_cmd = Node(
    # package='joint_state_publisher',
    # executable='joint_state_publisher',
    # name='joint_state_publisher',
    # condition=UnlessCondition(jsp_gui))

  # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
  # start_joint_state_publisher_gui_cmd = Node(
    # package='joint_state_publisher_gui',
    # executable='joint_state_publisher_gui',
    # name='joint_state_publisher_gui',
    # condition=IfCondition(jsp_gui))

  # # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  # robot_description_content = ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
  # start_robot_state_publisher_cmd = Node(
    # condition=IfCondition(use_robot_state_pub),
    # package='robot_state_publisher',
    # executable='robot_state_publisher',
    # name='robot_state_publisher',
    # output='screen',
    # parameters=[{
      # 'use_sim_time': use_sim_time, 
      # 'robot_description': robot_description_content}])

  # # Launch RViz
  # start_rviz_cmd = Node(
    # condition=IfCondition(use_rviz),
    # package='rviz2',
    # executable='rviz2',
    # name='rviz2',
    # output='screen',
    # arguments=['-d', rviz_config_file])  
    
  ############################ SPAWN #############################################
  # Define the spawn_entity node
  # spawn_entity_node = Node(
    # package='gazebo_ros',
    # executable='spawn_entity.py',
    # output='screen',
    # arguments=[
    #   '-entity', robot_name_in_model, 
    #   '-topic', 'robot_description',
    #   #'-file', urdf_model, 
    #   '-x', x, 
    #   '-y', y, 
    #   '-z', z, 
    #   '-R', roll,
    #   '-P', pitch,
    #   '-Y', yaw])

  # start_gazebo_ros_spawner_cmd = Node(
    # package='ros_gz_sim',
    # executable='create',
    # arguments=[
        # '-name', TURTLEBOT3_MODEL,
        # '-file', urdf_path,
        # '-x', x_pose,
        # '-y', y_pose,
        # '-z', '0.01'
    # ],
    # output='screen')
    
    # spawn = Node(package='ros_gz_sim', executable='create',
                 # parameters=[{
                    # 'name': 'my_custom_model',
                    # 'x': 1.2,
                    # 'z': 2.3,
                    # 'Y': 3.4,
                    # 'topic': '/robot_description'}],
                 # output='screen')

    # spawn = Node(
        # package='ros_gz_sim',
        # executable='create',
        # parameters=[{'name': 'rrbot',
                    # 'topic': 'robot_description'}],
        # output='screen',
    # )
 
  ############################ GZ-ROS BRIDGE #####################################
  # Bridge ROS topics and Gazebo messages for establishing communication
    # # Gz - ROS Bridge
    # bridge = Node(
        # package='ros_gz_bridge',
        # executable='parameter_bridge',
        # arguments=[
            # # Clock (IGN -> ROS2)
            # '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # # Joint states (IGN -> ROS2)
            # '/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        # ],
        # remappings=[
            # ('/world/empty/model/rrbot/joint_state', 'joint_states'),
        # ],
        # output='screen'
    # )
    
    
        # # Launch a bridge to forward tf and joint states to ros2
        # Node(
            # package='ros_gz_bridge',
            # executable='parameter_bridge',
            # arguments=[
                # '/world/default/model/double_pendulum_with_base0/joint_state@'
                # 'sensor_msgs/msg/JointState[gz.msgs.Model',
                # '/model/double_pendulum_with_base0/pose@'
                # 'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            # ],
            # remappings=[
                # ('/model/double_pendulum_with_base0/pose', '/tf'),
                # ('/world/default/model/double_pendulum_with_base0/joint_state', '/joint_states')
            # ]
        # ),
        
    # bridge_params = os.path.join(
        # get_package_share_directory('turtlebot3_gazebo'),
        # 'params',
        # 'turtlebot3_waffle_bridge.yaml'
    # )

    # start_gazebo_ros_bridge_cmd = Node(
        # package='ros_gz_bridge',
        # executable='parameter_bridge',
        # arguments=[
            # '--ros-args',
            # '-p',
            # f'config_file:={bridge_params}',
        # ],
        # output='screen',
    # )

    # bridge = Node(
        # package='ros_gz_bridge',
        # executable='parameter_bridge',
        # parameters=[{
            # 'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            # 'qos_overrides./tf_static.publisher.durability': 'transient_local',
        # }],
        # output='screen'
    # )
    
        # # Launch a bridge to forward tf and joint states to ros2
        # Node(
            # package='ros_gz_bridge',
            # executable='parameter_bridge',
            # arguments=[
                # '/world/default/model/double_pendulum_with_base0/joint_state@'
                # 'sensor_msgs/msg/JointState[gz.msgs.Model',
                # '/model/double_pendulum_with_base0/pose@'
                # 'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            # ],
            # remappings=[
                # ('/model/double_pendulum_with_base0/pose', '/tf'),
                # ('/world/default/model/double_pendulum_with_base0/joint_state', '/joint_states')
            # ]
        # ),
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  #ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  #ld.add_action(declare_urdf_model_path_cmd)
  #ld.add_action(declare_use_robot_state_pub_cmd)  
  #ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  #ld.add_action(declare_x_cmd)
  #ld.add_action(declare_y_cmd)
  #ld.add_action(declare_z_cmd)
  #ld.add_action(declare_roll_cmd)
  #ld.add_action(declare_pitch_cmd)
  #ld.add_action(declare_yaw_cmd)  

  # Add any actions
  ld.add_action(set_env_vars_resources)
  #ld.add_action(spawn_entity_node)
  #ld.add_action(gazebo_node)
  #ld.add_action(start_joint_state_publisher_gui_cmd)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  #ld.add_action(start_robot_state_publisher_cmd)
  #ld.add_action(start_rviz_cmd)

  return ld



