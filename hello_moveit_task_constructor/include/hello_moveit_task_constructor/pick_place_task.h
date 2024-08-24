/**
 * @file pick_place_task.hpp
 * @brief Defines the PickPlaceTask class for a pick and place demo using MoveIt Task Constructor.
 *
 * This file contains the declaration of the PickPlaceTask class, which sets up and executes
 * a pick and place task using MoveIt Task Constructor (MTC). It also includes a function
 * to set up the demo scene.
 *
 * @author Addison Sears-Collins
 * @date August 21, 2024
 */
#pragma once

// Include necessary ROS 2 headers
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

// Include MoveIt headers
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Include MoveIt Task Constructor (MTC) headers
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/properties.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

// Include the generated parameters header 
// (generated from src/pick_place_demo_parameters.yaml)
#include "pick_place_demo_parameters.hpp"

namespace hello_moveit_task_constructor {

// Using directive for convenience
using namespace moveit::task_constructor;

/**
 * @brief Set up the demo scene using ROS parameters.
 *
 * This function prepares the demo environment based on the provided parameters.
 *
 * @param params The parameters for the pick and place demo.
 */
void setupDemoScene(const pick_place_demo::Params& params);


/**
 * @brief Print detailed information about a stage and its substages.
 *
 * This function recursively prints information about a given stage and all of its substages.
 * It includes details such as the number of solutions, failures, and specific failure messages.
 * The output is indented to reflect the hierarchical structure of the stages.
 *
 * @param stage Pointer to the Stage object to be printed.
 * @param indent The indentation level for the current stage (default is 0).
 */
void printStageDetails(const moveit::task_constructor::Stage* stage, int indent = 0);

/**
 * @class PickPlaceTask
 * @brief Represents a pick and place task using MoveIt Task Constructor.
 *
 * This class encapsulates the functionality to set up, plan, and execute
 * a pick and place task using MoveIt Task Constructor.
 */
class PickPlaceTask
{
public:
  /**
   * @brief Construct a new PickPlaceTask object.
   *
   * @param task_name The name of the task.
   */
  PickPlaceTask(const std::string& task_name);

  /**
   * @brief Destroy the PickPlaceTask object.
   */
  ~PickPlaceTask() = default;

  /**
   * @brief Initialize the pick and place task.
   *
   * @param node The ROS 2 node.
   * @param params The parameters for the pick and place demo.
   * @return true if initialization was successful, false otherwise.
   */
  bool init(const rclcpp::Node::SharedPtr& node, const pick_place_demo::Params& params);

  /**
   * @brief Plan the pick and place task.
   *
   * @param max_solutions The maximum number of solutions to generate.
   * @return true if planning was successful, false otherwise.
   */
  bool plan(const std::size_t max_solutions);

  /**
   * @brief Execute the planned pick and place task.
   *
   * @return true if execution was successful, false otherwise.
   */
  bool execute();

private:
  // The name of the task
  std::string task_name_;

  // Pointer to the MoveIt Task Constructor task
  moveit::task_constructor::TaskPtr task_;
};

}  // namespace hello_moveit_task_constructor


