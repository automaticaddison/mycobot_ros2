cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Function to add an executable with common settings
function(add_common_executable target_name source_file)
  add_executable(${target_name} src/${source_file})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(${target_name}
    moveit_ros_planning_interface
    rclcpp
  )
endfunction()

# Add executables
add_common_executable(hello_moveit_v1 hello_moveit_v1.cpp)
add_common_executable(hello_moveit_v2 hello_moveit_v2.cpp)

# Install targets
install(TARGETS 
  hello_moveit_v1
  hello_moveit_v2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

