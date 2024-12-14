cmake_minimum_required(VERSION 3.8)
project(mycobot_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_visual_tools REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the .setup_assistant file
install(
  FILES .setup_assistant
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
