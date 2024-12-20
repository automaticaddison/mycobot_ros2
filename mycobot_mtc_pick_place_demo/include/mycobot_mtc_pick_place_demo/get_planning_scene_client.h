#ifndef GET_PLANNING_SCENE_CLIENT_H
#define GET_PLANNING_SCENE_CLIENT_H

/**
 * @file get_planning_scene_client.h
 * @brief Header file for the GetPlanningSceneClient class.
 *
 * This file declares the GetPlanningSceneClient class, which implements a ROS2 node
 * that acts as a client for the GetPlanningScene service. The class provides functionality
 * to request and process planning scene information, including collision objects,
 * point cloud data, and RGB image data.
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */


#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "mycobot_interfaces/srv/get_planning_scene.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "moveit_msgs/msg/planning_scene_world.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>


/**
 * @class GetPlanningSceneClient
 * @brief A ROS2 node that acts as a client for the GetPlanningScene service.
 *
 * This class provides functionality to request planning scene information from a ROS2 service
 * and process the response. It encapsulates the service client and methods to call the service
 * and handle the response.
 */
class GetPlanningSceneClient : public rclcpp::Node
{
public:
  /**
   * @struct PlanningSceneResponse
   * @brief Struct to encapsulate all fields from the service response.
   *
   * This struct holds various types of data received from the planning scene service.
   */
  struct PlanningSceneResponse {
    moveit_msgs::msg::PlanningSceneWorld scene_world; // The planning scene world information
    sensor_msgs::msg::PointCloud2 full_cloud; // The full point cloud data
    sensor_msgs::msg::Image rgb_image; // The RGB image data
    std::string target_object_id; // The ID of the target object
    std::string support_surface_id; // The ID of the support surface
    bool success; // Flag indicating whether the service call was successful
  };

  /**
   * @brief Constructor for the GetPlanningSceneClient class.
   *
   * Initializes the ROS2 node and sets up the service client.
   */
  GetPlanningSceneClient();

  /**
   * @brief Method to call the GetPlanningScene service and return the response.
   * @param target_shape The shape of the target object (e.g., "cylinder", "box").
   * @param target_dimensions The dimensions of the target object (e.g., [height, radius] for a cylinder).
   * @return PlanningSceneResponse containing the service response data.
   *
   * This method sends a request to the GetPlanningScene service and waits for a response.
   * It then processes the response and returns it as a PlanningSceneResponse struct.
   */
  PlanningSceneResponse call_service(const std::string& target_shape, const std::vector<double>& target_dimensions);

private:
  /**
   * @brief ROS2 service client for the GetPlanningScene service.
   *
   * This client is used to send requests to the GetPlanningScene service.
   */
  rclcpp::Client<mycobot_interfaces::srv::GetPlanningScene>::SharedPtr client_;

  /**
   * @brief Helper method to log information about the service response.
   * @param response The PlanningSceneResponse to log information about.
   *
   * This method prints out details about the received planning scene response,
   * which can be useful for debugging or monitoring the system's behavior.
   */
  void log_response_info(const PlanningSceneResponse& response);
};

#endif // GET_PLANNING_SCENE_CLIENT_H