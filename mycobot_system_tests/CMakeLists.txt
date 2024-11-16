cmake_minimum_required(VERSION 3.8)
project(mycobot_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

# Create executable
add_executable(arm_gripper_loop_controller src/arm_gripper_loop_controller.cpp)
ament_target_dependencies(arm_gripper_loop_controller rclcpp rclcpp_action control_msgs)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY scripts src
  DESTINATION share/${PROJECT_NAME}
)

# Install cpp executables
install(
  TARGETS
  arm_gripper_loop_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/arm_gripper_loop_controller.py
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

ament_package()
