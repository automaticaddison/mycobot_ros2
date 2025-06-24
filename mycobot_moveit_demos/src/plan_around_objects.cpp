/**
 * @file plan_around_objects.cpp
 * @brief Demonstrates basic usage of MoveIt with ROS 2 and collision avoidance.
 *
 * This program initializes a ROS 2 node, sets up a MoveIt interface for a robot manipulator,
 * creates a collision object in the scene, plans a motion to a target pose while avoiding
 * the collision object, and executes the planned motion.
 *
 * @author Addison Sears-Collins
 * @date December 15, 2024
 */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

/**
 * @brief The main function that starts our program.
 *
 * This function sets up our ROS 2 environment and prepares it for robot control.
 *
 * @param argc The number of input arguments our program receives.
 * @param argv The list of input arguments our program receives.
 * @return int A number indicating if our program finished successfully (0) or not.
 */
int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Creates a node named "plan_around_objects". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "plan_around_objects",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("plan_around_objects");

  // This code creates a separate thread, which is like a mini-program running alongside our main program.
  // This thread uses a ROS 2 "executor" to continuously process information about the robot's state.
  // By running this in its own thread, our ROS 2 node can keep getting updates about the robot without
  // interrupting the main flow of our program.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interfaces
  // These interfaces are used to plan and execute movements, set target poses,
  // and perform other motion-related tasks for each respective part of the robot.
  // The use of auto allows the compiler to automatically deduce the type of variable.
  // Source: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");

  // Specify a planning pipeline to be used for further planning
  arm_group_interface.setPlanningPipelineId("ompl");

  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");

  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(1.0);

  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(1.0);

  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              arm_group_interface.getRobotModel() };

  // Erase all existing visual markers from the RViz visualization.
  moveit_visual_tools.deleteAllMarkers();

  // Load the remote control interface for MoveIt visual tools.
  moveit_visual_tools.loadRemoteControl();

  // Lambda function to draw a title in the RViz visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    // Nested lambda to create a pose for the text
    auto const text_pose = [] {
        // Create an identity transform
        auto msg = Eigen::Isometry3d::Identity();
        // Set the z-coordinate to 1.0 (1 meter above the origin)
        msg.translation().z() = 1.0;
        return msg;
    }();

    // Publish the text to RViz
    // Parameters: pose, text content, color (WHITE), and size (XLARGE)
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  // Lambda function to display a prompt message in RViz
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  // Lambda function to visualize a trajectory as a line in RViz
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = arm_group_interface.getRobotModel()->getJointModelGroup("arm")](
	  auto const trajectory) {moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);};

  // Set a target pose for the end effector of the arm
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.128;
    msg.pose.position.y = -0.266;
    msg.pose.position.z = 0.111;
    msg.pose.orientation.x = 0.635;
    msg.pose.orientation.y = -0.268;
    msg.pose.orientation.z = 0.694;
    msg.pose.orientation.w = 0.206;
    return msg;
  }();
  arm_group_interface.setPoseTarget(arm_target_pose);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {

    // Print the planning frame for debugging purposes
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

    // Initialize a CollisionObject message
    moveit_msgs::msg::CollisionObject collision_object;

    // Set the frame ID for the collision object
    collision_object.header.frame_id = frame_id;

    // Set the timestamp to the current time
    collision_object.header.stamp = node->now();

    // Assign a unique ID to the collision object
    collision_object.id = "box1";

    // Define the shape of the collision object as a box
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    // Set the dimensions of the box (in meters)
    primitive.dimensions[primitive.BOX_X] = 0.2;  // Width
    primitive.dimensions[primitive.BOX_Y] = 0.05;  // Depth
    primitive.dimensions[primitive.BOX_Z] = 0.50;  // Height

    // Define the pose (position and orientation) of the box
    geometry_msgs::msg::Pose box_pose;

    // Set the position of the box center
    box_pose.position.x = 0.25;  // meters in x-direction
    box_pose.position.y = 0.0;   // Centered in y-direction
    box_pose.position.z = 0.25;  // meters in z-direction

    // Set the orientation of the box (no rotation in this case)
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;

    // Add the shape and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    // Set the operation to add the object to the planning scene
    collision_object.operation = collision_object.ADD;

    // Log information about the created collision object
    RCLCPP_INFO(logger, "Created collision object: %s", collision_object.id.c_str());

    RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f",
      primitive.dimensions[primitive.BOX_X],
      primitive.dimensions[primitive.BOX_Y],
      primitive.dimensions[primitive.BOX_Z]);

    RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
      box_pose.position.x, box_pose.position.y, box_pose.position.z);

    // Return the fully defined collision object
    return collision_object;
  }();

  // Set up a virtual representation of the robot's environment
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Add an object to this virtual environment that the robot needs to avoid colliding with
  planning_scene_interface.applyCollisionObject(collision_object);

  // Ask the user to press 'next' to continue
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");

  // Set up a title for the next visualization step
  draw_title("Planning");

  // Update the visualization to show the new title and wait for user input
  moveit_visual_tools.trigger();

  // Create a plan to that target pose
  // This will give us two things:
  // 1. Whether the planning was successful (stored in 'success')
  // 2. The actual motion plan (stored in 'plan')
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Try to execute the movement plan if it was created successfully
  // If the plan wasn't successful, report an error
  // Execute the plan
  if (success)
  {
	// Visualize the planned trajectory in RViz
    draw_trajectory_tool_path(plan.trajectory);

    // Trigger the visualization update
    moveit_visual_tools.trigger();

    // Prompt the user to continue to execution
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");

    // Update the title in RViz to show we're executing the plan
    draw_title("Executing");

    // Trigger another visualization update
    moveit_visual_tools.trigger();

    // Execute the planned motion
    arm_group_interface.execute(plan);
  }
  else
  {
    // If planning failed, update the title in RViz
    draw_title("Planning Failed!");

    // Trigger the visualization update
    moveit_visual_tools.trigger();

    // Log an error message
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Clean up and shut down the ROS 2 node
  rclcpp::shutdown();

  // Wait for the spinner thread to finish
  spinner.join();

  // Exit the program
  return 0;
}
