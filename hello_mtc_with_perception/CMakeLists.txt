# Minimum CMake version and project definition
cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(hello_mtc_with_perception)

# Build settings
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

# Define required packages (alphabetical order)
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

# Find required packages
foreach(package ${REQUIRED_PACKAGES})
  find_package(${package} REQUIRED)
endforeach()

# Find PCL components
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree search segmentation)

# Set dependencies
set(dependencies ${REQUIRED_PACKAGES})

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS})

# Libraries
## cluster_extraction
add_library(cluster_extraction SHARED
  src/cluster_extraction.cpp
)
target_include_directories(cluster_extraction PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cluster_extraction
  ${dependencies}
)
target_link_libraries(cluster_extraction
  ${PCL_LIBRARIES}
)

## get_planning_scene_client
add_library(get_planning_scene_client SHARED
  src/get_planning_scene_client.cpp
)
target_include_directories(get_planning_scene_client PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(get_planning_scene_client
  ${dependencies}
)

## normals_curvature_and_rsd_estimation
add_library(normals_curvature_and_rsd_estimation SHARED
  src/normals_curvature_and_rsd_estimation.cpp
)
target_include_directories(normals_curvature_and_rsd_estimation PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(normals_curvature_and_rsd_estimation
  ${dependencies}
)
target_link_libraries(normals_curvature_and_rsd_estimation
  ${PCL_LIBRARIES}
)

## plane_segmentation
add_library(plane_segmentation SHARED
  src/plane_segmentation.cpp
)
target_include_directories(plane_segmentation PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(plane_segmentation
  ${dependencies}
)
target_link_libraries(plane_segmentation
  ${PCL_LIBRARIES}
)

# Executables
## get_planning_scene_client_node
add_executable(get_planning_scene_client_node
  src/get_planning_scene_client.cpp
)
target_compile_definitions(get_planning_scene_client_node PRIVATE
  GET_PLANNING_SCENE_CLIENT_MAIN
)
target_include_directories(get_planning_scene_client_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(get_planning_scene_client_node
  ${dependencies}
)

## get_planning_scene_server
add_executable(get_planning_scene_server
  src/get_planning_scene_server.cpp
)
ament_target_dependencies(get_planning_scene_server
  ${dependencies}
)
target_link_libraries(get_planning_scene_server 
  cluster_extraction
  normals_curvature_and_rsd_estimation
  plane_segmentation
)

## mtc_node
add_executable(mtc_node
  src/mtc_node.cpp
)
ament_target_dependencies(mtc_node
  ${dependencies}
)
target_link_libraries(mtc_node
  get_planning_scene_client
)

# Installation
## Executables 
install(TARGETS 
  get_planning_scene_client_node
  get_planning_scene_server
  mtc_node 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Libraries
install(TARGETS 
  cluster_extraction
  get_planning_scene_client
  normals_curvature_and_rsd_estimation
  plane_segmentation     
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

## Directories 
install(DIRECTORY 
  config
  include/
  launch
  rviz
  scripts
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export and package
ament_export_include_directories(include)
ament_export_libraries(cluster_extraction get_planning_scene_client plane_segmentation)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()


