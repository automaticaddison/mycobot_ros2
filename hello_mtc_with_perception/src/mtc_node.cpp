/**
 * @file mtc_node.cpp
 * @brief Implementation of a MoveIt Task Constructor (MTC) node for a pick and place task.
 *
 * This program implements a pick and place task using MoveIt Task Constructor (MTC).
 * It creates a planning scene, generates a series of motion stages, and executes them
 * to pick up an object from one location and place it in another.
 *
 * @author Addison Sears-Collins
 * @date August 26, 2024
 */
 
// Include necessary ROS 2 and MoveIt headers
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

// Other utilities
#include <type_traits>
#include <string>
#include <vector>

// Conditional includes for tf2 geometry messages and Eigen
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace {
/**
 * @brief Transform a vector of numbers into a 3D position and orientation.
 * @param values Vector containing position and orientation values.
 * @return Eigen::Isometry3d representing the transformation.
 */
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

/**
 * @brief Convert a vector of numbers to a geometry_msgs::msg::Pose.
 * @param values Vector containing position and orientation values.
 * @return geometry_msgs::msg::Pose representing the pose.
 */
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
  return tf2::toMsg(vectorToEigen(values));
};
}  // namespace

// Namespace alias for MoveIt Task Constructor
namespace mtc = moveit::task_constructor;

/**
 * @brief Class representing the MTC Task Node.
 */
class MTCTaskNode : public rclcpp::Node
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  void doTask();
  void setupPlanningScene();

private:
  mtc::Task task_;
  mtc::Task createTask();
};

/**
 * @brief Constructor for the MTCTaskNode class.
 * @param options Node options for configuration.
 */
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : Node("mtc_node", options)
{
  auto declare_parameter = [this](const std::string& name, const auto& default_value, const std::string& description = "") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
      
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, default_value, descriptor);
    }
  };

  // General parameters
  declare_parameter("execute", false, "Whether to execute the planned task");
  declare_parameter("max_solutions", 10, "Maximum number of solutions to compute");
  declare_parameter("spawn_table", true, "Whether to spawn a table in the planning scene");

  // Controller parameters
  declare_parameter("controller_names", std::vector<std::string>{"arm_controller", "grip_action_controller"}, "Names of the controllers to use");

  // Robot configuration parameters
  declare_parameter("arm_group_name", "arm", "Name of the arm group in the SRDF");
  declare_parameter("gripper_group_name", "gripper", "Name of the gripper group in the SRDF");
  declare_parameter("gripper_frame", "link6_flange", "Name of the gripper frame");
  declare_parameter("gripper_open_pose", "open", "Name of the gripper open pose");
  declare_parameter("gripper_close_pose", "half_closed", "Name of the gripper closed pose");
  declare_parameter("arm_home_pose", "home", "Name of the arm home pose");

  // Scene frame parameters
  declare_parameter("world_frame", "base_link", "Name of the world frame");

  // Table parameters
  declare_parameter("table_name", "table", "Name of the table in the planning scene");
  declare_parameter("table_reference_frame", "base_link", "Reference frame for the table");
  declare_parameter("table_dimensions", std::vector<double>{0.10, 0.20, 0.03}, "Dimensions of the table [x, y, z]");
  declare_parameter("table_pose", std::vector<double>{0.22, 0.12, 0.0, 0.0, 0.0, 0.0}, "Pose of the table [x, y, z, roll, pitch, yaw]");

  // Object parameters
  declare_parameter("object_name", "object", "Name of the object to be manipulated");
  declare_parameter("object_reference_frame", "base_link", "Reference frame for the object");
  declare_parameter("object_dimensions", std::vector<double>{0.35, 0.0125}, "Dimensions of the object [height, radius]");
  declare_parameter("object_pose", std::vector<double>{0.22, 0.12, 0.0, 0.0, 0.0, 0.0}, "Initial pose of the object [x, y, z, roll, pitch, yaw]");

  // Grasp and place parameters
  declare_parameter("grasp_frame_transform", std::vector<double>{0.0, 0.0, 0.096, 1.5708, 0.0, 0.0}, "Transform from gripper frame to grasp frame [x, y, z, roll, pitch, yaw]");
  declare_parameter("place_pose", std::vector<double>{-0.183, -0.14, 0.0, 0.0, 0.0, 0.0}, "Pose where the object should be placed [x, y, z, roll, pitch, yaw]");
  declare_parameter("place_surface_offset", -0.03, "Offset from the surface when placing the object");

  // Motion planning parameters
  declare_parameter("approach_object_min_dist", 0.0015, "Minimum approach distance to the object");
  declare_parameter("approach_object_max_dist", 0.3, "Maximum approach distance to the object");
  declare_parameter("lift_object_min_dist", 0.005, "Minimum lift distance for the object");
  declare_parameter("lift_object_max_dist", 0.3, "Maximum lift distance for the object");
  declare_parameter("lower_object_min_dist", 0.005, "Minimum distance for lowering object");
  declare_parameter("lower_object_max_dist", 0.4, "Maximum distance for lowering object");

  // Timeout parameters
  declare_parameter("move_to_pick_timeout", 5.0, "Timeout for move to pick stage (seconds)");
  declare_parameter("move_to_place_timeout", 10.0, "Timeout for move to place stage (seconds)");

  // Grasp generation parameters
  declare_parameter("grasp_pose_angle_delta", 0.2618, "Angular resolution for sampling grasp poses (radians)");
  declare_parameter("grasp_pose_max_ik_solutions", 8, "Maximum number of IK solutions for grasp pose generation");
  declare_parameter("grasp_pose_min_solution_distance", 1.0, "Minimum distance in joint-space units between IK solutions for grasp pose");

  // Place generation parameters
  declare_parameter("place_pose_max_ik_solutions", 2, "Maximum number of IK solutions for place pose generation");

  // Cartesian planner parameters
  declare_parameter("cartesian_max_velocity_scaling", 1.0, "Max velocity scaling factor for Cartesian planner");
  declare_parameter("cartesian_max_acceleration_scaling", 1.0, "Max acceleration scaling factor for Cartesian planner");
  declare_parameter("cartesian_step_size", 0.00025, "Step size for Cartesian planner");

  // Direction vector parameters
  declare_parameter("approach_object_direction_z", 1.0, "Z component of approach object direction vector");
  declare_parameter("lift_object_direction_z", 1.0, "Z component of lift object direction vector");
  declare_parameter("lower_object_direction_z", -1.0, "Z component of lower object direction vector");
  declare_parameter("retreat_direction_z", -1.0, "Z component of retreat direction vector");

  // Other parameters
  declare_parameter("place_pose_z_offset_factor", 0.5, "Factor to multiply object height for place pose Z offset");
  declare_parameter("retreat_min_distance", 0.025, "Minimum distance for retreat motion");
  declare_parameter("retreat_max_distance", 0.25, "Maximum distance for retreat motion");

  RCLCPP_INFO(this->get_logger(), "All parameters have been declared with descriptions");
}

/**
 * @brief Set up the planning scene with collision objects.
 */
void MTCTaskNode::setupPlanningScene()
{
  // Create a planning scene interface to interact with the world
  moveit::planning_interface::PlanningSceneInterface psi;

  // Get general parameters
  auto spawn_table = this->get_parameter("spawn_table").as_bool();

  // Get table parameters
  auto table_name = this->get_parameter("table_name").as_string();
  auto table_dimensions = this->get_parameter("table_dimensions").as_double_array();
  auto table_pose_param = this->get_parameter("table_pose").as_double_array();
  auto table_reference_frame = this->get_parameter("table_reference_frame").as_string();

  if (spawn_table) {
    // Create a table collision object
    geometry_msgs::msg::Pose table_pose = vectorToPose(table_pose_param);
    moveit_msgs::msg::CollisionObject table_object;
    table_object.id = table_name;
    table_object.header.frame_id = table_reference_frame;
    table_object.primitives.resize(1);
    table_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table_object.primitives[0].dimensions = { table_dimensions.at(0), table_dimensions.at(1),
                                      table_dimensions.at(2) };
    table_pose.position.z -= 0.5 * table_dimensions[2]; // align surface with world
    table_object.primitive_poses.push_back(table_pose);

    // Add the table to the planning scene
    if (!psi.applyCollisionObject(table_object)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn table object: %s", table_object.id.c_str());
      throw std::runtime_error("Failed to spawn table object: " + table_object.id);
    }
    RCLCPP_INFO(this->get_logger(), "Added table to planning scene");
  } else {
    RCLCPP_INFO(this->get_logger(), "Skipping table spawn as per configuration");
  }

  // Get object parameters
  auto object_name = this->get_parameter("object_name").as_string();
  auto object_dimensions = this->get_parameter("object_dimensions").as_double_array();
  auto object_pose_param = this->get_parameter("object_pose").as_double_array();
  auto object_reference_frame = this->get_parameter("object_reference_frame").as_string();

  // Create a cylinder collision object
  geometry_msgs::msg::Pose cylinder_pose = vectorToPose(object_pose_param);
  auto place_pose_z_offset_factor = this->get_parameter("place_pose_z_offset_factor").as_double();
  cylinder_pose.position.z += place_pose_z_offset_factor * object_dimensions[0]; // Adjust z position before creating the object
  moveit_msgs::msg::CollisionObject cylinder_object;
  cylinder_object.id = object_name;
  cylinder_object.header.frame_id = object_reference_frame;
  cylinder_object.primitives.resize(1);
  cylinder_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder_object.primitives[0].dimensions = { object_dimensions.at(0), object_dimensions.at(1) };
  cylinder_object.primitive_poses.push_back(cylinder_pose);

  // Add the cylinder to the planning scene
  if (!psi.applyCollisionObject(cylinder_object)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn object: %s", cylinder_object.id.c_str());
    throw std::runtime_error("Failed to spawn object: " + cylinder_object.id);
  }
  RCLCPP_INFO(this->get_logger(), "Added object to planning scene");

  RCLCPP_INFO(this->get_logger(), "Planning scene setup completed");
}

/**
 * @brief Plan and/or execute the pick and place task.
 */
void MTCTaskNode::doTask()
{
  RCLCPP_INFO(this->get_logger(), "Starting the pick and place task");

  task_ = createTask();

  // Get parameters
  auto execute = this->get_parameter("execute").as_bool();
  auto max_solutions = this->get_parameter("max_solutions").as_int();

  try
  {
    task_.init();
    RCLCPP_INFO(this->get_logger(), "Task initialized successfully");
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
    return;
  }

  // Attempt to plan the task
  if (!task_.plan(max_solutions))
  {
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Task planning succeeded");

  // Publish the planned solution for visualization
  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(this->get_logger(), "Published solution for visualization");

  if (execute)
  {
    // Execute the planned task
    RCLCPP_INFO(this->get_logger(), "Executing the planned task");
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Task execution failed with error code: %d", result.val);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Task executed successfully");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Execution skipped as per configuration");
  }

  return;
}

/**
 * @brief Create the MTC task with all necessary stages.
 * @return The created MTC task.
 */
mtc::Task MTCTaskNode::createTask()
{
  RCLCPP_INFO(this->get_logger(), "Creating MTC task");
  
  // Create a new Task
  mtc::Task task;
  
  // Set the name of the task
  task.stages()->setName("pick_place_task");
  
  // Load the robot model into the task
  task.loadRobotModel(shared_from_this(), "robot_description");

  // Get parameters
  // Robot configuration parameters
  auto arm_group_name = this->get_parameter("arm_group_name").as_string();
  auto gripper_group_name = this->get_parameter("gripper_group_name").as_string();
  auto gripper_frame = this->get_parameter("gripper_frame").as_string();
  auto arm_home_pose = this->get_parameter("arm_home_pose").as_string();

  // Gripper poses
  auto gripper_open_pose = this->get_parameter("gripper_open_pose").as_string();
  auto gripper_close_pose = this->get_parameter("gripper_close_pose").as_string();

  // Frame parameters
  auto world_frame = this->get_parameter("world_frame").as_string();

  // Controller parameters
  auto controller_names = this->get_parameter("controller_names").as_string_array();

  // Object parameters
  auto object_name = this->get_parameter("object_name").as_string();
  auto object_reference_frame = this->get_parameter("object_reference_frame").as_string();
  auto object_dimensions = this->get_parameter("object_dimensions").as_double_array();
  auto object_pose = this->get_parameter("object_pose").as_double_array();

  // Table parameters
  auto table_name = this->get_parameter("table_name").as_string();
  auto table_reference_frame = this->get_parameter("table_reference_frame").as_string();

  // Grasp and place parameters
  auto grasp_frame_transform = this->get_parameter("grasp_frame_transform").as_double_array();
  auto place_pose = this->get_parameter("place_pose").as_double_array();
  auto place_surface_offset = this->get_parameter("place_surface_offset").as_double();

  // Motion planning parameters
  auto approach_object_min_dist = this->get_parameter("approach_object_min_dist").as_double();
  auto approach_object_max_dist = this->get_parameter("approach_object_max_dist").as_double();
  auto lift_object_min_dist = this->get_parameter("lift_object_min_dist").as_double();
  auto lift_object_max_dist = this->get_parameter("lift_object_max_dist").as_double();
  auto lower_object_min_dist = this->get_parameter("lower_object_min_dist").as_double();
  auto lower_object_max_dist = this->get_parameter("lower_object_max_dist").as_double();

  // Timeout parameters
  auto move_to_pick_timeout = this->get_parameter("move_to_pick_timeout").as_double();
  auto move_to_place_timeout = this->get_parameter("move_to_place_timeout").as_double();

  // Grasp generation parameters
  auto grasp_pose_angle_delta = this->get_parameter("grasp_pose_angle_delta").as_double();
  auto grasp_pose_max_ik_solutions = this->get_parameter("grasp_pose_max_ik_solutions").as_int();
  auto grasp_pose_min_solution_distance = this->get_parameter("grasp_pose_min_solution_distance").as_double();

  // Place generation parameters
  auto place_pose_max_ik_solutions = this->get_parameter("place_pose_max_ik_solutions").as_int();

  // Cartesian planner parameters
  auto cartesian_max_velocity_scaling = this->get_parameter("cartesian_max_velocity_scaling").as_double();
  auto cartesian_max_acceleration_scaling = this->get_parameter("cartesian_max_acceleration_scaling").as_double();
  auto cartesian_step_size = this->get_parameter("cartesian_step_size").as_double();

  // Direction vector parameters
  auto approach_object_direction_z = this->get_parameter("approach_object_direction_z").as_double();
  auto lift_object_direction_z = this->get_parameter("lift_object_direction_z").as_double();
  auto lower_object_direction_z = this->get_parameter("lower_object_direction_z").as_double();
  auto retreat_direction_z = this->get_parameter("retreat_direction_z").as_double();

  // Other parameters
  auto place_pose_z_offset_factor = this->get_parameter("place_pose_z_offset_factor").as_double();
  auto retreat_min_distance = this->get_parameter("retreat_min_distance").as_double();
  auto retreat_max_distance = this->get_parameter("retreat_max_distance").as_double();

  // Create planners for different types of motion
  // Pipeline planner for complex movements
  // OMPL planner
  std::unordered_map<std::string, std::string> ompl_map_arm = {
    {"ompl", arm_group_name + "[RRTConnectkConfigDefault]"}
  };
  auto ompl_planner_arm = std::make_shared<mtc::solvers::PipelinePlanner>(
    this->shared_from_this(),
    ompl_map_arm);
  RCLCPP_INFO(this->get_logger(), "OMPL planner created for the arm group");

  // JointInterpolation is a basic planner that is used for simple motions 
  // It computes quickly but doesn't support complex motions.
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  RCLCPP_INFO(this->get_logger(), "Joint Interpolation planner created for the gripper group");

  // Cartesian planner
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();  
  cartesian_planner->setMaxVelocityScalingFactor(cartesian_max_velocity_scaling);
  cartesian_planner->setMaxAccelerationScalingFactor(cartesian_max_acceleration_scaling);
  cartesian_planner->setStepSize(cartesian_step_size);
  RCLCPP_INFO(this->get_logger(), "Cartesian planner created");

  // Set task properties
  task.setProperty("trajectory_execution_info", 
    mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
  task.setProperty("group", arm_group_name); // The main planning group
  task.setProperty("eef", gripper_group_name); // The end-effector group
  task.setProperty("ik_frame", gripper_frame); // The frame for inverse kinematics

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  // Pointer to store the current state (will be used during the grasp pose generation stage)
  mtc::Stage* current_state_ptr = nullptr;

  // Add a stage to capture the current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  /****************************************************
   *                                                  *
   *               Open Gripper                       *
   *                                                  *
   ***************************************************/
  // This stage is responsible for opening the robot's gripper in preparation for picking 
  // up an object in the pick-and-place task. 
  auto stage_open_gripper =
    std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  stage_open_gripper->setGroup(gripper_group_name);  
  stage_open_gripper->setGoal(gripper_open_pose);
  stage_open_gripper->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
  task.add(std::move(stage_open_gripper));
  
  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/    
  // Create a stage to move the arm to a pre-grasp position
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group_name, ompl_planner_arm},
        {gripper_group_name, interpolation_planner}
      });
  stage_move_to_pick->setTimeout(move_to_pick_timeout);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));
  
  // Create a pointer for the stage that will attach the object (to be used later)
  // By declaring it at the top level of the function, it can be accessed throughout 
  // the entire task creation process. 
  // This allows different parts of the code to use and modify this pointer.
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator 
	  
  /****************************************************
   *                                                  *
   *               Pick Object                        *
   *                                                  *
   ***************************************************/
  {
    // Create a serial container for the grasping action
    // This container will hold stages (in order) that will accomplish the picking action
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

    /****************************************************
---- *               Approach Object                    *
     ***************************************************/
    {
      // Create a stage for moving the gripper close to the object before trying to grab it.	
      // We are doing a movement that is relative to our current position.	
      // Cartesian planner will move the gripper in a straight line	  
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner); 
  
      // Set properties for visualization and planning
      stage->properties().set("marker_ns", "approach_object"); // Namespace for visualization markers
      stage->properties().set("link", gripper_frame); // The link to move (end effector)
      stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" }); // Inherit the 'group' property
      stage->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);

      // Define the direction that we want the gripper to move (i.e. z direction) from the gripper frame
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = gripper_frame; // Set the frame for the vector
      vec.vector.z = approach_object_direction_z; // Set the direction (in this case, along the z-axis of the gripper frame)
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
	}
	
    /****************************************************
---- *               Generate Grasp Pose               *
     ***************************************************/
	{
	  // Generate the grasp pose
	  // This is the stage for computing how the robot should grab the object
	  // This stage is a generator stage because it doesn't need information from
	  // stages before or after it.
	  // When generating solutions, MTC will try to grab the object from many different orientations.
      // Sample grasp pose candidates in angle increments around the z-axis of the object
	  
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(gripper_open_pose);
      stage->setObject(object_name);
      stage->setAngleDelta(grasp_pose_angle_delta); //  Angular resolution for sampling grasp poses around the object
      stage->setMonitoredStage(current_state_ptr);  // Ensure grasp poses are valid given the initial configuration of the robot 

      // Compute IK for sampled grasp poses  
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(grasp_pose_max_ik_solutions);
      wrapper->setMinSolutionDistance(grasp_pose_min_solution_distance);
      wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame); // Transform from gripper frame to tool center point (TCP)
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
---- *            Allow Collision (gripper,  object)   *
     ***************************************************/
    {
      // Modify planning scene (w/o altering the robot's pose) to allow touching the object for picking
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions(
        object_name,
        task.getRobotModel()
        ->getJointModelGroup(gripper_group_name)
        ->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }
	
    /****************************************************
---- *               Close Gripper                     *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(gripper_group_name);
      stage->setGoal(gripper_close_pose);
      stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
      grasp->insert(std::move(stage));
    }
	
    /****************************************************
---- *               Attach Object                     *
     ***************************************************/
    {  
       auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
       stage->attachObject(object_name, gripper_frame);  // attach object to gripper_frame
       attach_object_stage = stage.get();
       grasp->insert(std::move(stage));
    }
	
    /****************************************************
---- *       Allow collision (object,  surface)        *
     ***************************************************/
    {
      // Allows the planner to generate valid trajectories where the object remains in contact 
      // with the support surface until it's lifted.
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({ object_name }, {table_name}, true);
      grasp->insert(std::move(stage));
    }
    /****************************************************
---- *       Lift object                               *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_object_min_dist, lift_object_max_dist);
      stage->setIKFrame(gripper_frame);
      stage->properties().set("marker_ns", "lift_object");
      stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
					  
      // We're defining the direction to lift the object
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = lift_object_direction_z;  // This means "straight up" 
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    /****************************************************
---- *       Forbid collision (object, surface)*       *
     ***************************************************/
    {
      // Forbid collisions between the picked object and the support surface. 
      // This is important after the object has been lifted to ensure it doesn't accidentally 
      // collide with the surface during subsequent movements.
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions({ object_name }, {table_name}, false);
      grasp->insert(std::move(stage));      
	}	 
	// Add the serial container to the robot's to-do list
	// This serial container contains all the sequential steps we've created for grasping
	// and lifting the object 
	task.add(std::move(grasp));
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
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place", 
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group_name, ompl_planner_arm},
        {gripper_group_name, interpolation_planner}
      });
    stage_move_to_place->setTimeout(move_to_place_timeout);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  /******************************************************
   *                                                    *
   *          Place Object                              *
   *                                                    *
   *****************************************************/
   // All placing sub-stages are collected within a serial container 
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    /******************************************************
---- *          Lower Object                              *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", gripper_frame);
      stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lower_object_min_dist, lower_object_max_dist);

      // Set downward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = lower_object_direction_z;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Generate Place Pose                       *
     *****************************************************/
    {
      // Generate Place Pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_name);

      // Set target pose
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = world_frame;
      target_pose_msg.pose = vectorToPose(place_pose);
      target_pose_msg.pose.position.z += place_pose_z_offset_factor * object_dimensions[0] + place_surface_offset;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // hook into successful pick solutions

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(place_pose_max_ik_solutions);
      wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame); // Transform from gripper frame to tool center point (TCP)
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" }); 
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
---- *          Open Gripper                              *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(gripper_group_name);
      stage->setGoal(gripper_open_pose);
      stage->properties().set("trajectory_execution_info",
        mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Forbid collision (gripper, object)        *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (gripper,object)");
      stage->allowCollisions(object_name, *task.getRobotModel()->getJointModelGroup(gripper_group_name),
        false);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Detach Object                             *
     *****************************************************/
    {
      // Update the planning scene to reflect that the object is no longer attached to the gripper.
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name, gripper_frame);
      place->insert(std::move(stage));
    }

    /******************************************************
---- *          Retreat Motion                            *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().set("trajectory_execution_info",
        mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(retreat_min_distance, retreat_max_distance);
      stage->setIKFrame(gripper_frame);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = gripper_frame;
      vec.vector.z = retreat_direction_z;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Add place container to task
    task.add(std::move(place));
  }

  /******************************************************
   *                                                    *
   *          Move to Home                              *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", ompl_planner_arm);
    stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(arm_home_pose);
    task.add(std::move(stage));
  }
  return task;
}

/**
 * @brief Main function to run the MTC task node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status.
 */
int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Set up node options
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // Create the MTC task node
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // Set up a multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node);

  // Set up the planning scene and execute the task
  try {
    RCLCPP_INFO(mtc_task_node->get_logger(), "Setting up planning scene");
    mtc_task_node->setupPlanningScene();
    RCLCPP_INFO(mtc_task_node->get_logger(), "Executing task");
    mtc_task_node->doTask();
    RCLCPP_INFO(mtc_task_node->get_logger(), "Task execution completed. Keeping node alive for visualization. Press Ctrl+C to exit.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(mtc_task_node->get_logger(), "An error occurred: %s", e.what());
  }

  // Keep the node running until Ctrl+C is pressed
  executor.spin();

  // Cleanup
  rclcpp::shutdown();

  return 0;
}
