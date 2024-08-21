cmake_minimum_required(VERSION 3.8)
project(hello_moveit_task_constructor)

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
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  tf2_eigen
)

# Create a shared library named hello_moveit_task_constructor_pick_place_task from the src/pick_place_task.cpp file
add_library(${PROJECT_NAME}_pick_place_task SHARED src/pick_place_task.cpp)
ament_target_dependencies(${PROJECT_NAME}_pick_place_task ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS ${PROJECT_NAME}_pick_place_task
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

include_directories(include)

# This function creates an executable for each demo, potentially associated with parameters (defined in a yaml file)
function(demo name)
  add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
  set(parameter_filename ${name}_parameters)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${parameter_filename}.yaml)
    generate_parameter_library(${parameter_filename} src/${parameter_filename}.yaml)
    target_link_libraries(${PROJECT_NAME}_${name} ${parameter_filename})
  endif()
  ament_target_dependencies(${PROJECT_NAME}_${name} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
  install(TARGETS ${PROJECT_NAME}_${name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

demo(alternative_path_costs)
demo(cartesian)
demo(fallbacks_move_to)
demo(ik_clearance_cost)
demo(modular)
demo(mtc_node_v1)
demo(pick_place_demo)

# This line below links the pick_place_demo_parameters library (pick_place_demo_parameters.yaml/pick_place_demo_parameters.hpp)
# to the hello_moveit_task_constructor_pick_place_task library (src/pick_place_task.cpp)
# Linking in this way explicitly declares dependencies between different parts of the project
target_link_libraries(${PROJECT_NAME}_pick_place_task pick_place_demo_parameters)

# This line below links the hello_moveit_task_constructor_pick_place_task library to 
# the hello_moveit_task_constructor_pick_place_demo executable (src/pick_place_demo.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_demo ${PROJECT_NAME}_pick_place_task)

# These lines above create a dependency chain:
#   - The pick_place_task library depends on the pick_place_demo_parameters library.
#   - The pick_place_demo executable depends on the pick_place_task library.

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

