/**
 * @file fallbacks_move_to.cpp
 * @brief Demonstrates using MoveIt Task Constructor for motion planning with fallback strategies.
 *
 * This program showcases how to use the MoveIt Task Constructor framework to create a motion
 * planning task with multiple initial states and fallback planning strategies. It plans a
 * movement for a robot arm using different planning methods (Cartesian, Pilz, and OMPL).
 *
 * Planning Methods:
 *   - Cartesian path
 *   - Pilz path
 *   - OMPL path
 *
 * @author Addison Sears-Collins
 * @date December 19, 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages.h>

// Define TAU as 2 * PI for easier angle calculations
constexpr double TAU = 2 * M_PI;

// Use the moveit::task_constructor namespace for convenience
using namespace moveit::task_constructor;

/**
 * @brief Main function to set up and run the MoveIt Task Constructor demo.
 *
 * This function demonstrates how to use the Fallbacks stage to try different planning approaches.
 * It sets up three different initial states and three planning methods (Cartesian, Pilz, and OMPL).
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * @return int Exit status of the program
 */
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

 // Declare the node parameters
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides({
    {"ompl.planning_plugin", "ompl_interface/OMPLPlanner"},
    {"pilz_industrial_motion_planner.planning_plugin", "pilz_industrial_motion_planner/CommandPlanner"}
  });

  // Create the node with the declared parameters
  auto node = rclcpp::Node::make_shared("fallbacks_move_to_demo", node_options);

  // Create a logger
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Initializing fallbacks_move_to_demo node");

  // Verify that the parameters are set
  std::string ompl_plugin, pilz_plugin;
  if (node->get_parameter("ompl.planning_plugin", ompl_plugin)) {
    RCLCPP_INFO(logger, "OMPL planning plugin: %s", ompl_plugin.c_str());
  } else {
    RCLCPP_ERROR(logger, "Failed to get OMPL planning plugin parameter");
  }
  if (node->get_parameter("pilz_industrial_motion_planner.planning_plugin", pilz_plugin)) {
    RCLCPP_INFO(logger, "Pilz planning plugin: %s", pilz_plugin.c_str());
  } else {
    RCLCPP_ERROR(logger, "Failed to get Pilz planning plugin parameter");
  }

  // Create a separate thread for spinning the node
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  // Set up the main Task
  Task t;
  t.setName("fallback strategies in MoveTo");
  t.loadRobotModel(node);
  const moveit::core::RobotModelConstPtr robot{ t.getRobotModel() };

  // Ensure we're using the correct robot model
  assert(robot->getName() == "mycobot_280");
  RCLCPP_INFO(logger, "Robot model loaded: %s", robot->getName().c_str());

  // Set up different path planning methods

  // Cartesian path planner (lowest computational requirements, best for straight-line paths with no obstacles)
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setJumpThreshold(2.0);
  RCLCPP_INFO(logger, "Cartesian path planner set up with jump threshold: 2.0");

  // Create PipelinePlanner for Pilz (moderate computational requirements, inherently considers obstacles)
  // Found via -> ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces "{}"
  std::unordered_map<std::string, std::string> pilz_map = {
    {"pilz_industrial_motion_planner", "PTP"}
  };
  auto pilz_planner = std::make_shared<solvers::PipelinePlanner>(node, pilz_map);
  RCLCPP_INFO(logger, "Pilz planner created");

  // Create PipelinePlanner for OMPL (high computational requirements, best for complex paths with many obstacles)
  std::unordered_map<std::string, std::string> ompl_map = {
    {"ompl", "arm[RRTConnectkConfigDefault]"}
  };
  auto ompl_planner = std::make_shared<solvers::PipelinePlanner>(node, ompl_map);
  RCLCPP_INFO(logger, "OMPL planner created");

  // Define the target end state for all task plans
  std::map<std::string, double> target_state;
  robot->getJointModelGroup("arm")->getVariableDefaultPositions("ready", target_state);
  target_state["link1_to_link2"] = +TAU / 8;
  RCLCPP_INFO(logger, "Target state set for 'arm' group");

  // Define the default initial state
  RCLCPP_INFO(logger, "Setting up initial scene");
  auto initial_scene{ std::make_shared<planning_scene::PlanningScene>(robot) };
  initial_scene->getCurrentStateNonConst().setToDefaultValues(robot->getJointModelGroup("arm"), "ready");

  // Set up three different initial states using an Alternatives container
  RCLCPP_INFO(logger, "Setting up initial state alternatives");
  auto initial_alternatives = std::make_unique<Alternatives>("initial states");

  // First initial state option: 90 degree offset from target goal
  {
    auto fixed{ std::make_unique<stages::FixedState>("90 degree offset from target goal") };
    auto scene{ initial_scene->diff() };
    scene->getCurrentStateNonConst().setVariablePositions({ { "link1_to_link2", -TAU / 8 } });
    fixed->setState(scene);
    initial_alternatives->add(std::move(fixed));
  }

  // Second initial state option: directly reachable without collision
  {
    auto fixed{ std::make_unique<stages::FixedState>("directly reachable without collision") };
    auto scene{ initial_scene->diff() };
    scene->getCurrentStateNonConst().setVariablePositions({
      { "link1_to_link2", +TAU / 8 },
      { "link3_to_link4", 0 },
    });
    fixed->setState(scene);
    initial_alternatives->add(std::move(fixed));
  }

  // Third initial state option: getting to target requires collision avoidance
  {
    auto fixed{ std::make_unique<stages::FixedState>("getting to target requires collision avoidance") };
    auto scene{ initial_scene->diff() };
    scene->getCurrentStateNonConst().setVariablePositions({ { "link1_to_link2", -TAU / 8 } });

    // Add a collision object (box) to the scene
    scene->processCollisionObjectMsg([]() {
      moveit_msgs::msg::CollisionObject co;
      co.id = "box";
      co.header.frame_id = "base_link";
      co.operation = co.ADD;
      co.pose = []() {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.02;
        p.position.y = -0.20;
        p.position.z = 0.32 / 2;
        p.orientation.w = 1.0;
        return p;
      }();
      co.primitives.push_back([]() {
        shape_msgs::msg::SolidPrimitive sp;
        sp.type = sp.BOX;
        sp.dimensions = { 0.005, 0.1, 0.32 };
        return sp;
      }());
      return co;
    }());
    fixed->setState(scene);
    initial_alternatives->add(std::move(fixed));
  }

  // Add the initial states to the task
  RCLCPP_INFO(logger, "Adding initial states to the task");
  t.add(std::move(initial_alternatives));

  // Set up fallback strategies to reach the target state
  RCLCPP_INFO(logger, "Setting up fallback strategies");
  auto fallbacks = std::make_unique<Fallbacks>("move to other side");

  // Helper lambda to add different planning methods to the fallbacks container
  auto add_to_fallbacks{ [&](auto& solver, auto& name) {
    auto move_to = std::make_unique<stages::MoveTo>(name, solver);
    move_to->setGroup("arm");
    move_to->setGoal(target_state);
    fallbacks->add(std::move(move_to));
  } };

  // Add different planning methods to the fallbacks container
  add_to_fallbacks(cartesian, "Cartesian path");
  add_to_fallbacks(pilz_planner, "Pilz path");
  add_to_fallbacks(ompl_planner, "OMPL path");

  // Add the fallback strategies to the task
  RCLCPP_INFO(logger, "Adding fallback strategies to the task");
  t.add(std::move(fallbacks));

  // Plan the task
  RCLCPP_INFO(logger, "Starting task planning");
  try {
    t.plan();
    RCLCPP_INFO(logger, "Task planning completed successfully");
  } catch (const InitStageException& e) {
    RCLCPP_ERROR(logger, "InitStageException caught: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Exception caught: %s", e.what());
  }

  // Wait for the spinning thread to finish (keeps the program running for RViz inspection)
  spinning_thread.join();

  return 0;
}



