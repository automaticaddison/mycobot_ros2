/**
 * @file modular.cpp
 * @brief Demonstrates the use of MoveIt Task Constructor for robot motion planning.
 *
 * This program creates a reusable task for a robot arm using MoveIt Task Constructor.
 * It defines a series of movements including Cartesian paths and joint space motions.
 *
 * Key Concept:
 *   SerialContainer: This is a type of container in MoveIt Task Constructor that holds
 *     multiple movement stages. These stages are executed in sequence, one after another.
 *     Think of it like a to-do list for the robot, where each item must be completed
 *     before moving on to the next one.
 *
 * @author Addison Sears-Collins
 * @date December 19, 2024
 */

// Include necessary headers
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/container.h>
#include <moveit/planning_scene/planning_scene.hpp>

// Use the moveit::task_constructor namespace for convenience
using namespace moveit::task_constructor;

/**
 * @brief Creates a reusable module for robot movement.
 *
 * @param group The name of the robot group to move.
 * @return std::unique_ptr<SerialContainer> A container with a series of movement stages.
 */
std::unique_ptr<SerialContainer> createModule(const std::string& group) {
  // Create a new SerialContainer to hold our movement stages
  auto c = std::make_unique<SerialContainer>("Cartesian Path");
  c->setProperty("group", group);

  RCLCPP_INFO(rclcpp::get_logger("modular_demo"), "Creating module for group: %s", group.c_str());

  // Create solvers for Cartesian and joint space planning
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

  // Stage 1: Move 5 cm in the positive X direction
  {
    auto stage = std::make_unique<stages::MoveRelative>("x +0.05", cartesian);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.x = 0.05;
    stage->setDirection(direction);
    c->insert(std::move(stage));
    RCLCPP_INFO(rclcpp::get_logger("modular_demo"), "Added stage: Move 5 cm in +X direction");
  }

  // Stage 2: Move 2 cm in the negative Y direction
  {
    auto stage = std::make_unique<stages::MoveRelative>("y -0.02", cartesian);
    stage->properties().configureInitFrom(Stage::PARENT);
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.y = -0.02;
    stage->setDirection(direction);
    c->insert(std::move(stage));
    RCLCPP_INFO(rclcpp::get_logger("modular_demo"), "Added stage: Move 2 cm in -Y direction");
  }

  // Stage 3: Rotate -18 degrees around the Z axis
  {
    auto stage = std::make_unique<stages::MoveRelative>("rz -18°", cartesian);
    stage->properties().configureInitFrom(Stage::PARENT);
    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = "base_link";
    twist.twist.angular.z = -M_PI / 10.; // 18 degrees in radians
    stage->setDirection(twist);
    c->insert(std::move(stage));
    RCLCPP_INFO(rclcpp::get_logger("modular_demo"), "Added stage: Rotate -18 degrees around Z axis");
  }

  // Stage 4: Move to the "ready" position
  {
    auto stage = std::make_unique<stages::MoveTo>("moveTo ready", joint_interpolation);
    stage->properties().configureInitFrom(Stage::PARENT);
    stage->setGoal("ready");
    c->insert(std::move(stage));
    RCLCPP_INFO(rclcpp::get_logger("modular_demo"), "Added stage: Move to 'ready' position");
  }

  RCLCPP_INFO(rclcpp::get_logger("modular_demo"), "Module creation completed with 4 stages");
  return c;
}

/**
 * @brief Creates the main task for robot movement.
 *
 * @param node The ROS2 node to use for loading the robot model.
 * @return Task The complete task for robot movement.
 */
Task createTask(const rclcpp::Node::SharedPtr& node) {
  Task t;
  t.loadRobotModel(node);
  t.stages()->setName("Reusable Containers");

  RCLCPP_INFO(node->get_logger(), "Creating task: %s", t.stages()->name().c_str());

  // Add the current state as the starting point
  t.add(std::make_unique<stages::CurrentState>("current"));
  RCLCPP_INFO(node->get_logger(), "Added current state as starting point");

  // Define the robot group to move
  const std::string group = "arm";

  // Add a stage to move to the "ready" position
  {
    auto stage = std::make_unique<stages::MoveTo>("move to ready", std::make_shared<solvers::JointInterpolationPlanner>());
    stage->setGroup(group);
    stage->setGoal("ready");
    t.add(std::move(stage));
    RCLCPP_INFO(node->get_logger(), "Added stage: Move to 'ready' position");
  }

  // Add five instances of our reusable module
  // This creates a sequence of movements that the robot will perform,
  // repeating the same set of actions five times in a row.
  RCLCPP_INFO(node->get_logger(), "Adding 5 instances of the reusable module");
  for (int i = 1; i <= 5; ++i) {
    t.add(createModule(group));
    RCLCPP_INFO(node->get_logger(), "Added module instance %d", i);
  }

  // Add a stage to move to the "home" position
  {
    auto stage = std::make_unique<stages::MoveTo>("move to home", std::make_shared<solvers::JointInterpolationPlanner>());
    stage->setGroup(group);
    stage->setGoal("home");
    t.add(std::move(stage));
    RCLCPP_INFO(node->get_logger(), "Added stage: Move to 'home' position");
  }

  RCLCPP_INFO(node->get_logger(), "Task creation completed with 5 module instances");
  return t;
}

/**
 * @brief Main function to set up and execute the robot task.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("modular_demo");
  auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Starting modular demo");

  // Start a separate thread for ROS2 spinning
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  // Create and plan the task
  auto task = createTask(node);
  try {
    RCLCPP_INFO(logger, "Starting task planning");

    // Plan the task
    moveit::core::MoveItErrorCode error_code = task.plan();

    // Log the planning result
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Task planning completed successfully");
      RCLCPP_INFO(logger, "Found %zu solutions", task.numSolutions());

      // Use printState to log the task state
      std::ostringstream state_stream;
      task.printState(state_stream);
      RCLCPP_INFO(logger, "Task state:\n%s", state_stream.str().c_str());

      // If planning succeeds, publish the solution
      task.introspection().publishSolution(*task.solutions().front());
      RCLCPP_INFO(logger, "Published solution");
    } else {
      RCLCPP_ERROR(logger, "Task planning failed with error code: %d", error_code.val);

      // Use explainFailure to log the reason for failure
      std::ostringstream failure_stream;
      task.explainFailure(failure_stream);
      RCLCPP_ERROR(logger, "Failure explanation:\n%s", failure_stream.str().c_str());
    }

    // Log a simple summary of each stage
    RCLCPP_INFO(logger, "Stage summary:");
    for (size_t i = 0; i < task.stages()->numChildren(); ++i) {
      const auto* stage = task.stages()->operator[](i);
      RCLCPP_INFO(logger, "  %s: %zu solutions, %zu failures",
                  stage->name().c_str(), stage->solutions().size(), stage->failures().size());
    }

  } catch (const InitStageException& ex) {
    RCLCPP_ERROR(logger, "InitStageException caught during task planning: %s", ex.what());
    std::ostringstream oss;
    oss << task;
    RCLCPP_ERROR(logger, "Task details:\n%s", oss.str().c_str());
  }

  RCLCPP_INFO(logger, "Modular demo completed");

  // Wait for the spinning thread to finish
  spinning_thread.join();

  return 0;
}

