/**
 * @file pick_place_task.cpp
 * @brief Implementation of the PickPlaceTask class for a pick and place operation.
 *
 * This file contains the implementation of the PickPlaceTask class, which sets up and executes
 * a pick and place task using MoveIt Task Constructor. It includes stages for picking up an object,
 * moving it to a new location, and placing it down.
 *
 * @author Addison Sears-Collins
 * @date August 21, 2024
 */

#include <Eigen/Geometry>
#include <hello_moveit_task_constructor/pick_place_task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "pick_place_demo_parameters.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

namespace {
// Transform a vector of numbers into a 3D position and orientation
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

// Convert a vector of numbers to a geometry_msgs::msg::Pose
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
  return tf2::toMsg(vectorToEigen(values));
};
}  // namespace

namespace hello_moveit_task_constructor {

// Spawn an object in the planning scene
void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi,
                 const moveit_msgs::msg::CollisionObject& object) {
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to spawn object: " + object.id);
}

// Create a table collision object
moveit_msgs::msg::CollisionObject createTable(const pick_place_demo::Params& params) {
  geometry_msgs::msg::Pose pose = vectorToPose(params.table_pose);
  moveit_msgs::msg::CollisionObject object;
  object.id = params.table_name;
  object.header.frame_id = params.table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { params.table_dimensions.at(0), params.table_dimensions.at(1),
                                      params.table_dimensions.at(2) };
  pose.position.z -= 0.5 * params.table_dimensions[2];  // align surface with world
  object.primitive_poses.push_back(pose);
  return object;
}

// Create a cylinder collision object
moveit_msgs::msg::CollisionObject createObject(const pick_place_demo::Params& params) {
  geometry_msgs::msg::Pose pose = vectorToPose(params.object_pose);
  moveit_msgs::msg::CollisionObject object;
  object.id = params.object_name;
  object.header.frame_id = params.object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { params.object_dimensions.at(0), params.object_dimensions.at(1) };
  pose.position.z += 0.5 * params.object_dimensions[0];
  object.primitive_poses.push_back(pose);
  return object;
}

// Set up the demo scene with table and cylinder
void setupDemoScene(const pick_place_demo::Params& params) {
  // Add table and cylinder to planning scene
  rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
  moveit::planning_interface::PlanningSceneInterface psi;
  if (params.spawn_table)
    spawnObject(psi, createTable(params));
  spawnObject(psi, createObject(params));
}

/**
 * @brief Print detailed information about a stage and its substages.
 * @param stage Pointer to the Stage object to be printed.
 * @param indent The indentation level for the current stage (default is 0).
 */
void printStageDetails(const moveit::task_constructor::Stage* stage, int indent) {
  std::string indentation(indent * 2, ' ');
  RCLCPP_INFO(LOGGER, "%s%s: %zu solutions, %zu failures",
              indentation.c_str(), stage->name().c_str(), 
              stage->solutions().size(), stage->numFailures());

  // Print solution details
  for (const auto& solution : stage->solutions()) {
    RCLCPP_INFO(LOGGER, "%s  Solution: cost = %.3f, comment = %s",
                indentation.c_str(), solution->cost(), solution->comment().c_str());
  }

  // Print failure details
  for (const auto& failure : stage->failures()) {
    RCLCPP_WARN(LOGGER, "%s  Failure: %s", indentation.c_str(), failure->comment().c_str());
  }

  // Check if the stage is a container and has child stages
  if (const auto* container = dynamic_cast<const moveit::task_constructor::ContainerBase*>(stage)) {
    size_t num_children = container->numChildren();
    for (size_t i = 0; i < num_children; ++i) {
      Stage* child = (*container)[i];
      printStageDetails(child, indent + 1);
    }

    // Additional information for specific container types
    if (dynamic_cast<const moveit::task_constructor::SerialContainer*>(container)) {
      RCLCPP_INFO(LOGGER, "%sType: Serial Container", indentation.c_str());
    } else if (dynamic_cast<const moveit::task_constructor::Alternatives*>(container)) {
      RCLCPP_INFO(LOGGER, "%sType: Alternatives Container", indentation.c_str());
    } else if (dynamic_cast<const moveit::task_constructor::Fallbacks*>(container)) {
      RCLCPP_INFO(LOGGER, "%sType: Fallbacks Container", indentation.c_str());
    } else if (dynamic_cast<const moveit::task_constructor::Merger*>(container)) {
      RCLCPP_INFO(LOGGER, "%sType: Merger Container", indentation.c_str());
    } else if (dynamic_cast<const moveit::task_constructor::WrapperBase*>(container)) {
      RCLCPP_INFO(LOGGER, "%sType: Wrapper", indentation.c_str());
    }
  }
}

/**
 * @brief Constructor for the PickPlaceTask class.
 * @param task_name The name of the task.
 */
PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

/**
 * @brief Initializes the pick and place task.
 * @param node Shared pointer to the ROS2 node.
 * @param params Parameters for the pick and place task.
 * @return true if initialization is successful, false otherwise.
 */
bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node, const pick_place_demo::Params& params) {
	RCLCPP_INFO(LOGGER, "Initializing task pipeline");

	// Reset before constructing the new task object
        task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	// Individual movement stages are collected within the Task object
	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel(node);

	/* Create planners used in various stages. Various options are available,
	namely OMPL, Cartesian, etc. */
	// OMPL planner
	std::unordered_map<std::string, std::string> ompl_map_arm = {
	{"ompl", params.arm_group_name + "[RRTConnectkConfigDefault]"}
	};
	auto ompl_planner_arm = std::make_shared<solvers::PipelinePlanner>(node, ompl_map_arm);
	RCLCPP_INFO(LOGGER, "OMPL planner created for the arm group");
	
	// JointInterpolation is a basic planner that is used for simple motions 
	// It computes quickly but doesn't support complex motions.
	auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();
	RCLCPP_INFO(LOGGER, "Joint Interpolation planner created for the gripper group");
	
	// Create an OMPL planner for the gripper (use if JointInterpolation doesn't work)
	std::unordered_map<std::string, std::string> ompl_map_gripper = {
	{"ompl", params.gripper_group_name + "[RRTConnectkConfigDefault]"}
	};
	auto ompl_planner_gripper = std::make_shared<solvers::PipelinePlanner>(node, ompl_map_gripper);
	RCLCPP_INFO(LOGGER, "OMPL planner created for the gripper group");
	
	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(1.0);
	cartesian_planner->setMaxAccelerationScalingFactor(1.0);
	cartesian_planner->setStepSize(.0005);
	RCLCPP_INFO(LOGGER, "Cartesian planner created");

	// Set task properties
	t.setProperty("group", params.arm_group_name);
	t.setProperty("eef", params.eef_name);
	t.setProperty("gripper", params.gripper_group_name);
	t.setProperty("gripper_grasping_frame", params.gripper_frame);
	t.setProperty("ik_frame", params.gripper_frame);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		// Check the current state and ensure the object is not already attached
		// Predicate Filter is a task stage that checks whether certain conditions are met before 
		// allowing a task to continue.
		// This filter acts as a gatekeeper to ensure that the pick-and-place task does not attempt 
		// to pick up an object that is already attached to the robot 
		auto current_state = std::make_unique<stages::CurrentState>("current state");
		auto applicability_filter =
		  std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object = params.object_name](const SolutionBase& s, std::string& comment) {
		  if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
                    comment = "object with id '" + object + "' is already attached and cannot be picked";
                    return false;
                  }
                  return true;
		});
		t.add(std::move(applicability_filter));
	}

	/****************************************************
	 *                                                  *
	 *               Open Gripper                       *
	 *                                                  *
	 ***************************************************/
    	// This stage is responsible for opening the robot's gripper in preparation for picking 
    	// up an object in the pick-and-place task. 
	Stage* initial_state_ptr = nullptr;
	{
		auto stage = std::make_unique<stages::MoveTo>("open gripper", interpolation_planner);
		stage->setGroup(params.gripper_group_name);
		stage->setGoal(params.gripper_open_pose);
		initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	// The purpose of the "move to pick" stage is to move the robot's arm from its current position 
	// (after the gripper has opened) to a position where it can start the pick operation. 
	// This involves calculating and executing a valid trajectory using the ompl_planner_arm.
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { params.arm_group_name, ompl_planner_arm } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/
	Stage* pick_stage_ptr = nullptr; // Store a reference to the entire pick/grasp stage of the task.
	{
		// A SerialContainer combines several sub-stages, here for picking the object
		// ensures that the sub-stages within the container will use consistent property values 
        	// that are inherited from the overarching task configuration.
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "gripper", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "gripper", "group", "ik_frame" });

		/****************************************************
  ---- *               Approach Object                    *
		 ***************************************************/
		{
			// Move the eef link forward along its z-axis by an amount within the given min-max range
            		// The MoveRelative stage is typically used when you want to move the robot by a certain distance or 
            		// direction relative to its current pose, rather than moving it to an absolute target position.
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", params.gripper_frame);  // link to perform IK for
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });  // inherit group from parent stage
			stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);

			// Set gripper forward direction
            		// This sets the direction in which the gripper will move. Here, the code specifies that the movement 
            		// should be in the positive Z direction of the gripper's reference frame 
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.gripper_frame;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
			// Sample grasp pose candidates in angle increments around the z-axis of the object
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->setPreGraspPose(params.gripper_open_pose);
			stage->setObject(params.object_name);  // object to sample grasps for
			stage->setAngleDelta(M_PI / 12); //  Angular resolution for sampling grasp poses around the object
			stage->setMonitoredStage(initial_state_ptr);  // Ensure grasp poses are valid given the initial configuration of the robot 

			// Compute IK for sampled grasp poses
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);  // limit number of solutions to reduce computational overhead
			wrapper->setMinSolutionDistance(1.0); // Solutions must be distinct from each other
			// define virtual frame to reach the target_pose
			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.gripper_frame); // Intended grasp point
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });  // inherit properties from parent
			wrapper->properties().configureInitFrom(Stage::INTERFACE,
			                                        { "target_pose" });  // inherit property from child solution
			grasp->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (gripper,  object) *
		 ***************************************************/
		{
			// Modify planning scene (w/o altering the robot's pose) to allow touching the object for picking
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (gripper,object)");
			stage->allowCollisions(
			    params.object_name,
			    t.getRobotModel()->getJointModelGroup(params.gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Close Gripper                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close gripper", interpolation_planner);
			stage->setGroup(params.gripper_group_name);
			stage->setGoal(params.gripper_close_pose);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(params.object_name, params.gripper_frame);  // attach object to gripper_frame
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Allow collision (object,  surface) *
		 ***************************************************/
		{
			// Allows the planner to generate valid trajectories where the object remains in contact 
			// with the support surface until it's lifted.
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ params.object_name }, { params.surface_link }, true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Lift object                        *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist); // Min and max distance for lifting 
			stage->setIKFrame(params.gripper_frame);
			stage->properties().set("marker_ns", "lift_object");

			// Set upward direction for lifting the object
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.world_frame;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Forbid collision (object, surface)*
		 ***************************************************/
		{
            		// Forbid collisions between the picked object and the support surface. 
            		// This is important after the object has been lifted to ensure it doesn't accidentally 
            		// collide with the surface during subsequent movements.
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ params.object_name }, { params.surface_link }, false);
			grasp->insert(std::move(stage));
		}

		pick_stage_ptr = grasp.get();  // Monitor the success of the pick operation when generating place poses

		// Add grasp container to task
		t.add(std::move(grasp));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		// Connect the grasped state to the pre-place state, i.e. realize the object transport
		// In other words, this stage plans the motion that transports the object from where it was picked up 
		// to where it will be placed.
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { params.arm_group_name, ompl_planner_arm } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Place Object                              *
	 *                                                    *
	 *****************************************************/
	// All placing sub-stages are collected within a serial container again
	{
		auto place = std::make_unique<SerialContainer>("place object");
		t.properties().exposeTo(place->properties(), { "eef", "gripper", "group" });
		place->properties().configureInitFrom(Stage::PARENT, { "eef", "gripper", "group" });

		/******************************************************
  ---- *          Lower Object                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
			stage->properties().set("marker_ns", "lower_object");
			stage->properties().set("link", params.gripper_frame);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.01, .3);

			// Set downward direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.world_frame;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Generate Place Pose                       *
		 *****************************************************/
		{
			// Generate Place Pose
			auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
			stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
			stage->properties().set("marker_ns", "place_pose");
			stage->setObject(params.object_name);

			// Set target pose
			geometry_msgs::msg::PoseStamped p;
			p.header.frame_id = params.object_reference_frame;
			p.pose = vectorToPose(params.place_pose);
			p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
			stage->setPose(p);
			stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(2);
			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.gripper_frame);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" }); 
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		/******************************************************
  ---- *          Open Gripper                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("open gripper", interpolation_planner);
			stage->setGroup(params.gripper_group_name);
			stage->setGoal(params.gripper_open_pose);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Forbid collision (gripper, object)        *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (gripper,object)");
			stage->allowCollisions(params.object_name, *t.getRobotModel()->getJointModelGroup(params.gripper_group_name),
			                       false);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Detach Object                             *
		 *****************************************************/
		{
            		// Update the planning scene to reflect that the object is no longer attached to the gripper.
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(params.object_name, params.gripper_frame);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Retreat Motion                            *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.12, .25);
			stage->setIKFrame(params.gripper_frame);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.gripper_frame;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		// Add place container to task
		t.add(std::move(place));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Home                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", ompl_planner_arm);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal(params.arm_home_pose);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}

	// prepare Task structure for planning
	try {
		t.init();
	} catch (InitStageException& e) {
		RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
		return false;
	}

	return true;
}

/**
 * @brief Plans the pick and place task.
 * @param max_solutions Maximum number of solutions to compute.
 * @return true if planning is successful, false otherwise.
 */
bool PickPlaceTask::plan(const std::size_t max_solutions) {
  RCLCPP_INFO(LOGGER, "Start searching for task solutions");
  try {
    moveit::core::MoveItErrorCode error_code = task_->plan(max_solutions);
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Task planning completed successfully. Found %zu solutions", task_->numSolutions());
      
      if (!task_->solutions().empty()) {
        RCLCPP_INFO(LOGGER, "Publishing solution for visualization");
        task_->introspection().publishSolution(*task_->solutions().front());
      }
      // Print detailed stage summary
      RCLCPP_INFO(LOGGER, "Detailed stage summary:");
      printStageDetails(task_->stages(), 0);  // Pass 0 as the initial indentation level
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Task planning failed with error code: %d", error_code.val);
      
      // Print detailed failure explanation
      std::ostringstream explanation;
      task_->explainFailure(explanation);
      RCLCPP_ERROR(LOGGER, "Failure explanation: %s", explanation.str().c_str());
      
      // Print detailed stage summary even if planning failed
      RCLCPP_INFO(LOGGER, "Detailed stage summary:");
      printStageDetails(task_->stages(), 0);  // Pass 0 as the initial indentation level
      return false;
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(LOGGER, "Exception during planning: %s", ex.what());
    return false;
  }
}
/**
 * @brief Executes the planned pick and place task.
 * @return true if execution is successful, false otherwise.
 */
bool PickPlaceTask::execute() {
  RCLCPP_INFO(LOGGER, "Starting execution of solution trajectory");

  if (task_->solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "No solutions available to execute");
    return false;
  }

  const auto& solution = task_->solutions().front();
  RCLCPP_INFO(LOGGER, "Executing solution");

  moveit::core::MoveItErrorCode execute_result;

  try {
    execute_result = task_->execute(*solution);

    if (execute_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Task execution completed successfully");
    } else {
      RCLCPP_ERROR(LOGGER, "Task execution failed with error code: %d", execute_result.val);
      
      // Log additional details about the failed execution
      RCLCPP_ERROR(LOGGER, "Failed solution details:");
      RCLCPP_ERROR(LOGGER, "  Cost: %f", solution->cost());
      if (!solution->comment().empty()) {
        RCLCPP_ERROR(LOGGER, "  Comment: %s", solution->comment().c_str());
      }
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(LOGGER, "Exception during task execution: %s", ex.what());
    return false;
  }

  return (execute_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}
}  // namespace hello_moveit_task_constructor
