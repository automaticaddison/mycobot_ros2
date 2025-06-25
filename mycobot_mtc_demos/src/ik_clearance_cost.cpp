/**
 * @file ik_clearance_cost.cpp
 * @brief Demonstrates using MoveIt Task Constructor for motion planning with collision avoidance.
 *
 * This program sets up a motion planning task for a mycobot_280 robot arm using MoveIt Task Constructor.
 * It creates a scene with an obstacle, computes inverse kinematics (IK) solutions, and plans a motion
 * while considering clearance from obstacles.
 *
 * @author Addison Sears-Collins
 * @date December 19, 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/cost_terms.h>
#include <mycobot_mtc_demos/ik_clearance_cost_parameters.hpp>

// Use the moveit::task_constructor namespace for convenience
using namespace moveit::task_constructor;

/* ComputeIK(FixedState) */
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  auto node = rclcpp::Node::make_shared("ik_clearance_cost_demo");

  // Create a logger
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Starting IK Clearance Cost Demo");

  // Start a separate thread to handle ROS 2 callbacks
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  // Create a parameter listener for IK clearance cost parameters
  const auto param_listener = std::make_shared<ik_clearance_cost_demo::ParamListener>(node);
  const auto params = param_listener->get_params();
  RCLCPP_INFO(logger, "Parameters loaded: cumulative=%s, with_world=%s",
              params.cumulative ? "true" : "false",
              params.with_world ? "true" : "false");

  // Create a Task object to hold the planning stages
  Task t;
  t.stages()->setName("clearance IK");
  RCLCPP_INFO(logger, "Task created: %s", t.stages()->name().c_str());

  // Wait for 500 milliseconds to ensure ROS 2 is fully initialized
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Load the robot model (mycobot_280)
  t.loadRobotModel(node);
  assert(t.getRobotModel()->getName() == "mycobot_280");
  RCLCPP_INFO(logger, "Robot model loaded: %s", t.getRobotModel()->getName().c_str());

  // Create a planning scene
  auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
  RCLCPP_INFO(logger, "Planning scene created");

  // Set the robot to its default state
  auto& robot_state = scene->getCurrentStateNonConst();
  robot_state.setToDefaultValues();
  RCLCPP_INFO(logger, "Robot state set to default values");

  // Set the arm to its "ready" position
  [[maybe_unused]] bool found =
      robot_state.setToDefaultValues(robot_state.getJointModelGroup("arm"), "ready");
  assert(found);
  RCLCPP_INFO(logger, "Arm set to 'ready' position");

  // Create an obstacle in the scene
  moveit_msgs::msg::CollisionObject co;
  co.id = "obstacle";
  co.primitives.emplace_back();
  co.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  co.primitives[0].dimensions.resize(1);
  co.primitives[0].dimensions[0] = 0.1;
  co.header.frame_id = t.getRobotModel()->getModelFrame();
  co.primitive_poses.emplace_back();
  co.primitive_poses[0].orientation.w = 1.0;
  co.primitive_poses[0].position.x = -0.183;
  co.primitive_poses[0].position.y = -0.14;
  co.primitive_poses[0].position.z = 0.15;
  scene->processCollisionObjectMsg(co);
  RCLCPP_INFO(logger, "Obstacle added to scene: sphere at position (%.2f, %.2f, %.2f) with radius %.2f",
              co.primitive_poses[0].position.x, co.primitive_poses[0].position.y,
              co.primitive_poses[0].position.z, co.primitives[0].dimensions[0]);

  // Create a FixedState stage to set the initial state
  auto initial = std::make_unique<stages::FixedState>();
  initial->setState(scene);
  initial->setIgnoreCollisions(true);
  RCLCPP_INFO(logger, "FixedState stage created");

  // Create a ComputeIK stage for inverse kinematics
  auto ik = std::make_unique<stages::ComputeIK>();
  ik->insert(std::move(initial));
  ik->setGroup("arm");

  // Set the target pose
  ik->setTargetPose(Eigen::Translation3d(-.183, 0.0175, .15) * Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitX()));

  ik->setTimeout(1.0);
  ik->setMaxIKSolutions(100);

  // Set up the clearance cost term
  auto cl_cost{ std::make_unique<cost::Clearance>() };
  cl_cost->cumulative = params.cumulative;
  cl_cost->with_world = params.with_world;
  ik->setCostTerm(std::move(cl_cost));
  RCLCPP_INFO(logger, "Clearance cost term added to ComputeIK stage");

  // Add the ComputeIK stage to the task
  t.add(std::move(ik));
  RCLCPP_INFO(logger, "ComputeIK stage added to task");

  // Attempt to plan the task
  try {
    RCLCPP_INFO(logger, "Starting task planning");

    // Plan the task
    moveit::core::MoveItErrorCode error_code = t.plan(0);

    // Log the planning result
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Task planning completed successfully");
      RCLCPP_INFO(logger, "Found %zu solutions", t.numSolutions());

      // Use printState to log the task state
      std::ostringstream state_stream;
      t.printState(state_stream);
      RCLCPP_INFO(logger, "Task state:\n%s", state_stream.str().c_str());

    } else {
      RCLCPP_ERROR(logger, "Task planning failed with error code: %d", error_code.val);

      // Use explainFailure to log the reason for failure
      std::ostringstream failure_stream;
      t.explainFailure(failure_stream);
      RCLCPP_ERROR(logger, "Failure explanation:\n%s", failure_stream.str().c_str());
    }

    // Log a simple summary of each stage
    RCLCPP_INFO(logger, "Stage summary:");
    for (size_t i = 0; i < t.stages()->numChildren(); ++i) {
      const auto* stage = t.stages()->operator[](i);
      RCLCPP_INFO(logger, "  %s: %zu solutions, %zu failures",
                  stage->name().c_str(), stage->solutions().size(), stage->failures().size());
    }

  } catch (const InitStageException& e) {
    RCLCPP_ERROR(logger, "InitStageException caught during task planning: %s", e.what());
  }

  RCLCPP_INFO(logger, "IK Clearance Cost Demo completed");

  // Wait for the spinning thread to finish
  spinning_thread.join();

  return 0;
}

