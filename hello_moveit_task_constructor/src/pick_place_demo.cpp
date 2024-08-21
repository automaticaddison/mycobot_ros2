/**
 * @file pick_place_demo.cpp
 * @brief Main entry point for the pick and place demo using MoveIt Task Constructor.
 *
 * This program sets up and runs a pick and place task demonstration using the
 * MoveIt Task Constructor framework. It initializes the ROS 2 node, sets up the
 * demo scene, plans the pick and place task, and optionally executes it.
 *
 * @author Addison Sears-Collins
 * @date August 21, 2024
 */

#include <rclcpp/rclcpp.hpp>

// Include the pick/place task implementation
#include <hello_moveit_task_constructor/pick_place_task.h>
#include "pick_place_demo_parameters.hpp" // Automatically generated from the yaml file pick_place_demo_parameters.yaml 

// Set up a logger for this demo
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

int main(int argc, char** argv) {
	
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set up node options to automatically declare parameters
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Create a ROS 2 node for this demo
  auto node = rclcpp::Node::make_shared("pick_place_demo", node_options);

  // Start a separate thread to handle ROS 2 callbacks
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  // Create a parameter listener to get the demo parameters
  const auto param_listener = std::make_shared<pick_place_demo::ParamListener>(node);
  const auto params = param_listener->get_params();

  // Set up the demo scene based on the parameters
  hello_moveit_task_constructor::setupDemoScene(params);

  // Create the pick and place task
  hello_moveit_task_constructor::PickPlaceTask pick_place_task("pick_place_task");

  // Initialize the pick and place task
  if (!pick_place_task.init(node, params)) {
    RCLCPP_INFO(LOGGER, "Initialization failed");
    return 1;
  }

  // Plan the pick and place task
  if (pick_place_task.plan(params.max_solutions)) {
    RCLCPP_INFO(LOGGER, "Planning succeeded");

    // Execute the plan if execution is enabled in the parameters
    if (params.execute) {
      pick_place_task.execute();
      RCLCPP_INFO(LOGGER, "Execution complete");
    } else {
      RCLCPP_INFO(LOGGER, "Execution disabled");
    }
  } else {
    RCLCPP_INFO(LOGGER, "Planning failed");
  }

  // Wait for the spinning thread to finish (keeps the node alive for introspection)
  spinning_thread.join();

  return 0;
}


