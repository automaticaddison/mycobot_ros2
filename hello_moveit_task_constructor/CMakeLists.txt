cmake_minimum_required(VERSION 3.8)
project(hello_moveit_task_constructor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

# Include the 'include' directory
include_directories(include)

add_executable(mtc_node_v1 src/mtc_node_v1.cpp)

target_include_directories(mtc_node_v1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(mtc_node_v1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  mtc_node_v1
  "generate_parameter_library"
  "moveit_core"
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "rclcpp"
)

install(TARGETS mtc_node_v1
  DESTINATION lib/${PROJECT_NAME})
  
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
