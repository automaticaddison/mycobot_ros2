/**
 * @file get_planning_scene_client.cpp
 * @brief Implementation of the GetPlanningSceneClient class and standalone executable.
 *
 * This file contains the implementation of the GetPlanningSceneClient class,
 * which acts as a client for the GetPlanningScene service in ROS2.
 * It can be compiled as part of a library or as a standalone executable.
 *
 * @author Addison Sears-Collins
 * @date September 19, 2024
 */

#include "hello_mtc_with_perception/get_planning_scene_client.h"

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

    RCLCPP_INFO(get_logger(), "  Number of planes: %zu", obj.planes.size());
    for (size_t i = 0; i < obj.planes.size(); ++i) {
      const auto& plane = obj.planes[i];
      const auto& plane_pose = obj.plane_poses[i];
      RCLCPP_INFO(get_logger(), "    Plane %zu:", i);
      RCLCPP_INFO(get_logger(), "      Coefficients: a=%.3f, b=%.3f, c=%.3f, d=%.3f",
                  plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3]);
      RCLCPP_INFO(get_logger(), "      Pose: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                  plane_pose.position.x, plane_pose.position.y, plane_pose.position.z,
                  plane_pose.orientation.x, plane_pose.orientation.y, plane_pose.orientation.z, plane_pose.orientation.w);
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

