cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(hello_mtc_with_perception)

# Set build type and C++ standard
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

set(REQUIRED_PACKAGES
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  mycobot_interfaces
  PCL
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
)

foreach(package ${REQUIRED_PACKAGES})
  find_package(${package} REQUIRED)
endforeach()

# Find PCL components
find_package(PCL REQUIRED COMPONENTS common filters io kdtree search segmentation features)

# Set dependencies
set(dependencies ${REQUIRED_PACKAGES})

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS})

# Create plane_segmentation as a shared library
add_library(plane_segmentation SHARED src/plane_segmentation.cpp)
target_link_libraries(plane_segmentation ${PCL_LIBRARIES})
target_include_directories(plane_segmentation PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(plane_segmentation ${dependencies})

# Define and build mtc_node
add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node ${dependencies})

# Define and build get_planning_scene_server
add_executable(get_planning_scene_server src/get_planning_scene_server.cpp)
ament_target_dependencies(get_planning_scene_server ${dependencies})
target_link_libraries(get_planning_scene_server plane_segmentation)

# Install targets
install(
  TARGETS mtc_node get_planning_scene_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS plane_segmentation
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch rviz scripts urdf DESTINATION share/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(plane_segmentation)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
