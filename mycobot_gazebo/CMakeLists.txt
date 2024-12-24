cmake_minimum_required(VERSION 3.8)
project(mycobot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gripper_controllers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
