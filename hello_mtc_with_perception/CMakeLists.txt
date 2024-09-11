cmake_minimum_required(VERSION 3.8)
project(hello_mtc_with_perception)

# Set the build type if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(mycobot_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  filters
  io
  kdtree
  search
  segmentation
  features
)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Set dependencies
set(dependencies
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  mycobot_interfaces
  PCL
  pcl_conversions
  rclcpp
  sensor_msgs
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
)

# Include cpp "include" directory to the include path
include_directories(
  include
)

# Create Cpp executables
add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node ${dependencies})

add_executable(get_planning_scene_server src/get_planning_scene_server.cpp)
ament_target_dependencies(get_planning_scene_server ${dependencies})

# Copy folders into the install/ folder of the ROS 2 workspace
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY config launch scripts urdf 
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(
  TARGETS
  get_planning_scene_server
  mtc_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})

ament_package()

