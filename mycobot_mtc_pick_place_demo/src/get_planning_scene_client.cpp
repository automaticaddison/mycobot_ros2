/**
 * @file get_planning_scene_client.cpp
 * @brief Implementation of the GetPlanningSceneClient class and standalone executable.
 *
 * This file contains the implementation of the GetPlanningSceneClient class,
 * which acts as a client for the GetPlanningScene service in ROS2.
 * It can be compiled as part of a library or as a standalone executable.
 *
 * Input:
 * - target_shape (std::string): The shape of the target object (e.g., "cylinder", "box")
 * - target_dimensions (std::vector<double>): The dimensions of the target object
 *
 * Output:
 * - PlanningSceneResponse struct containing:
 *   - scene_world (moveit_msgs::msg::PlanningSceneWorld): The planning scene world
 *   - full_cloud (sensor_msgs::msg::PointCloud2): The full point cloud of the scene
 *   - rgb_image (sensor_msgs::msg::Image): The RGB image of the scene
 *   - target_object_id (std::string): The ID of the target object in the scene
 *   - support_surface_id (std::string): The ID of the support surface in the scene
 *   - success (bool): Indicates if the service call was successful
 *
 * The client sends a request to the GetPlanningScene service and processes the response,
 * providing detailed logging of the received planning scene information.
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

#include "mycobot_mtc_pick_place_demo/get_planning_scene_client.h"

GetPlanningSceneClient::GetPlanningSceneClient()
: Node("get_planning_scene_client")
{
  // Create a client for the GetPlanningScene service
  client_ = create_client<mycobot_interfaces::srv::GetPlanningScene>("/get_planning_scene_mycobot");
}

GetPlanningSceneClient::PlanningSceneResponse
GetPlanningSceneClient::call_service(const std::string& target_shape, const std::vector<double>& target_dimensions)
{
  PlanningSceneResponse response;
  response.success = false;

  // Wait for the service to become available
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return response;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  // Prepare the request
  auto request = std::make_shared<mycobot_interfaces::srv::GetPlanningScene::Request>();
  request->target_shape = target_shape;
  request->target_dimensions = target_dimensions;

  // Send the request asynchronously
  auto result_future = client_->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    RCLCPP_INFO(get_logger(), "Service call successful");

    // Store the result in our response struct
    response.scene_world = result->scene_world;
    response.full_cloud = result->full_cloud;
    response.rgb_image = result->rgb_image;
    response.target_object_id = result->target_object_id;
    response.support_surface_id = result->support_surface_id;
    response.success = result->success;

    // Log information about the response
    log_response_info(response);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to call service");
  }

  return response;
}

void GetPlanningSceneClient::log_response_info(const PlanningSceneResponse& response)
{
  RCLCPP_INFO(get_logger(), "Response received. Success: %s", response.success ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Target object ID: %s", response.target_object_id.c_str());
  RCLCPP_INFO(get_logger(), "Support surface ID: %s", response.support_surface_id.c_str());
  RCLCPP_INFO(get_logger(), "Number of collision objects: %zu", response.scene_world.collision_objects.size());
  RCLCPP_INFO(get_logger(), "Point cloud width: %d, height: %d", response.full_cloud.width, response.full_cloud.height);
  RCLCPP_INFO(get_logger(), "RGB image width: %d, height: %d", response.rgb_image.width, response.rgb_image.height);

  // Detailed logging for each collision object
  for (const auto& obj : response.scene_world.collision_objects) {
    RCLCPP_INFO(get_logger(), "Collision object details:");
    RCLCPP_INFO(get_logger(), "  ID: %s", obj.id.c_str());

    RCLCPP_INFO(get_logger(), "  Number of primitives: %zu", obj.primitives.size());
    for (size_t i = 0; i < obj.primitives.size(); ++i) {
      const auto& prim = obj.primitives[i];
      const auto& prim_pose = obj.primitive_poses[i];
      RCLCPP_INFO(get_logger(), "    Primitive %zu:", i);
      RCLCPP_INFO(get_logger(), "      Type: %d", prim.type);
      RCLCPP_INFO(get_logger(), "      Dimensions: %zu", prim.dimensions.size());
      for (size_t j = 0; j < prim.dimensions.size(); ++j) {
        RCLCPP_INFO(get_logger(), "        Dimension %zu: %.3f", j, prim.dimensions[j]);
      }
      RCLCPP_INFO(get_logger(), "      Pose: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                  prim_pose.position.x, prim_pose.position.y, prim_pose.position.z,
                  prim_pose.orientation.x, prim_pose.orientation.y, prim_pose.orientation.z, prim_pose.orientation.w);
    }

  }
}


// Conditional compilation for standalone executable
#ifdef GET_PLANNING_SCENE_CLIENT_MAIN

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create an instance of our GetPlanningSceneClient
  auto node = std::make_shared<GetPlanningSceneClient>();

  // Example input parameters
  std::string target_shape = "cylinder";
  std::vector<double> target_dimensions = {0.35, 0.0125};

  // Call the service
  auto response = node->call_service(target_shape, target_dimensions);

  // Process the response
  if (response.success) {
    RCLCPP_INFO(node->get_logger(), "Service call completed successfully");
    // Additional processing can be done here
  } else {
    RCLCPP_WARN(node->get_logger(), "Service response incomplete or invalid. Check get_planning_scene_server logs for the explanation.");
  }

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}

#endif // GET_PLANNING_SCENE_CLIENT_MAIN