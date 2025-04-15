/**
 * @file get_planning_scene_server.cpp
 * @brief ROS 2 service for processing point cloud data to generate a MoveIt planning scene.
 *
 * This program implements a ROS 2 service that processes point cloud and RGB image data
 * to generate CollisionObjects for a MoveIt planning scene. It segments the input point cloud,
 * fits primitive shapes to the segments, creates corresponding CollisionObjects, and provides
 * the necessary data for subsequent grasp generation.
 *
 * Service:
 *     get_planning_scene_mycobot (mycobot_interfaces/srv/GetPlanningScene):
 *         Processes point cloud data and returns a planning scene
 *
 * Subscription Topics:
 *     [point_cloud_topic] (sensor_msgs/PointCloud2): Input point cloud data
 *     [rgb_image_topic] (sensor_msgs/Image): Input RGB image data
 *
 * Service Input:
 *     target_shape (string): Target object shape (e.g., "cylinder", "box")
 *     target_dimensions (vector<double>): Approximate target object dimensions
 *
 * Service Output:
 *     scene_world (moveit_msgs/PlanningSceneWorld): Contains CollisionObjects for all detected objects
 *     full_cloud (sensor_msgs/PointCloud2): Full scene point cloud
 *     rgb_image (sensor_msgs/Image): RGB image of the scene
 *     target_object_id (string): ID of the target object in the PlanningSceneWorld
 *     support_surface_id (string): ID of the support surface in the PlanningSceneWorld
 *     success (bool): Indicates if the operation was successful
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "mycobot_mtc_pick_place_demo/cluster_extraction.h"
#include "mycobot_mtc_pick_place_demo/normals_curvature_and_rsd_estimation.h"
#include "mycobot_mtc_pick_place_demo/object_segmentation.h"
#include "mycobot_mtc_pick_place_demo/plane_segmentation.h"
#include <mycobot_interfaces/srv/get_planning_scene.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

class GetPlanningSceneServer : public rclcpp::Node {
 public:
  GetPlanningSceneServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("get_planning_scene_server", options) {
    declareParameters();
    createSubscribers();
    createService();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

 private:
  // Parameters for inputs
  std::string point_cloud_topic;
  std::string rgb_image_topic;
  std::string target_frame;

  // Cropping parameters
  bool enable_cropping;
  double crop_min_x;
  double crop_max_x;
  double crop_min_y;
  double crop_max_y;
  double crop_min_z;
  double crop_max_z;

  // Parameters for plane and object segmentation
  int max_iterations;
  double distance_threshold;
  double z_tolerance;
  double angle_tolerance;
  double plane_segmentation_threshold;
  int min_cluster_size;
  int max_cluster_size;
  double cluster_tolerance;
  int normal_estimation_k;
  double w_inliers;
  double w_size;
  double w_distance;
  double w_orientation;
  int max_plane_segmentation_iterations;
  double plane_segmentation_distance_threshold;

  // Parameters for creating the support plane
  std::string support_surface_name;
  double min_surface_thickness;

  // Parameters for normal, curvature, and RSD estimation
  int k_neighbors;
  double max_plane_error;
  int max_iterations_normals;
  int min_boundary_neighbors;
  double rsd_radius;
  pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr cloud_with_features;

  // Parameters for cluster extraction
  int nearest_neighbors;
  float smoothness_threshold;
  float curvature_threshold;

  // Parameters for object segmentation
  int num_iterations;
  int inlier_threshold;
  int hough_radius_bins;
  int hough_center_bins;
  double ransac_distance_threshold;
  int ransac_max_iterations;
  // Parameters for circle filtering
  int circle_min_cluster_size;
  int circle_max_clusters;
  double circle_height_tolerance;
  double circle_curvature_threshold;
  double circle_radius_tolerance;
  double circle_normal_angle_threshold;
  double circle_cluster_tolerance;
  // Parameters for line filtering
  int line_min_cluster_size;
  int line_max_clusters;
  double line_curvature_threshold;
  double line_cluster_tolerance;
  double line_rho_threshold;
  double line_theta_threshold;

  // Legacy...remove later Shape fitting parameters
  int shape_fitting_max_iterations;
  double shape_fitting_distance_threshold;
  double shape_fitting_min_radius;
  double shape_fitting_max_radius;
  double shape_fitting_normal_distance_weight;
  double shape_fitting_normal_search_radius;

  // For output pcd (point cloud) files
  std::string output_directory;
  std::string debug_pcd_filename;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub;

  // Service
  rclcpp::Service<mycobot_interfaces::srv::GetPlanningScene>::SharedPtr service;

  // Latest data storage
  sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud;
  sensor_msgs::msg::Image::SharedPtr latest_rgb_image;

  // TF2 objects
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void declareParameters() {
    auto declare_parameter = [this](const std::string& name, const auto& default_value, const std::string& description = "") {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = description;

      if (!this->has_parameter(name)) {
        this->declare_parameter(name, default_value, descriptor);
      }
    };

    // Declare parameters with default values and descriptions
    declare_parameter("point_cloud_topic", "/camera_head/depth/color/points", "Topic name for incoming point cloud data");
    declare_parameter("rgb_image_topic", "/camera_head/color/image_raw", "Topic name for incoming RGB image data");
    declare_parameter("target_frame", "base_link", "Target frame for the planning scene");

    // Declare cropping parameters
    declare_parameter("enable_cropping", true, "Enable or disable point cloud cropping");
    declare_parameter("crop_min_x", 0.10, "Minimum X value for crop box");
    declare_parameter("crop_max_x", 1.1, "Maximum X value for crop box");
    declare_parameter("crop_min_y", 0.00, "Minimum Y value for crop box");
    declare_parameter("crop_max_y", 0.90, "Maximum Y value for crop box");
    declare_parameter("crop_min_z", -std::numeric_limits<double>::infinity(), "Minimum Z value for crop box");
    declare_parameter("crop_max_z", std::numeric_limits<double>::infinity(), "Maximum Z value for crop box");

    // Declare parameters for plane and object segmentation
    declare_parameter("max_iterations", 100, "Maximum iterations for RANSAC");
    declare_parameter("distance_threshold", 0.01, "Distance threshold for RANSAC");
    declare_parameter("z_tolerance", 0.03, "Tolerance for z-coordinate of the support plane");
    declare_parameter("angle_tolerance", 0.9990482216, "Angle tolerance for surface normals");
    declare_parameter("plane_segmentation_threshold", 0.001, "Threshold for plane segmentation");
    declare_parameter("min_cluster_size", 100, "Minimum size of a cluster");
    declare_parameter("max_cluster_size", 1000000, "Maximum size of a cluster");
    declare_parameter("cluster_tolerance", 0.02, "Tolerance for Euclidean clustering");
    declare_parameter("normal_estimation_k", 30, "Number of neighbors for normal estimation");
    declare_parameter("w_inliers", 1.0, "Weight for inlier score in plane model selection");
    declare_parameter("w_size", 1.0, "Weight for size score in plane model selection");
    declare_parameter("w_distance", 1.0, "Weight for distance score in plane model selection");
    declare_parameter("w_orientation", 1.0, "Weight for orientation score in plane model selection");
    declare_parameter("max_plane_segmentation_iterations", 1000, "Maximum iterations for plane segmentation RANSAC");
    declare_parameter("plane_segmentation_distance_threshold", 0.01, "Distance threshold for plane segmentation (in meters)");

    // Declare parameters for creating the support surface
    declare_parameter("support_surface_name", "support_surface", "Name of the support surface collision object");
    declare_parameter("min_surface_thickness", 0.0001, "Minimum thickness for the support surface (in meters)");

    // Declare parameters for normal, curvature, and RSD estimation
    declare_parameter("k_neighbors", 30, "Number of neighbors to consider for normal estimation");
    declare_parameter("max_plane_error", 0.01, "Threshold for MLESAC plane fitting");
    declare_parameter("max_iterations_normals", 100, "Maximum iterations for MLESAC in normal estimation");
    declare_parameter("min_boundary_neighbors", 10, "Minimum number of neighbors for boundary points");
    declare_parameter("rsd_radius", 0.01, "Radius for RSD estimation");

    // Declare new parameters for cluster extraction
    declare_parameter("nearest_neighbors", 30, "Number of neighbors to consider for region growing");
    declare_parameter("smoothness_threshold", 20.0f, "Smoothness threshold for the region growing algorithm (in degrees)");
    declare_parameter("curvature_threshold", 0.2f, "Curvature threshold for the region growing algorithm");

    // Declare new parameters for object segmentation
    declare_parameter("num_iterations", 5, "Number of iterations for the inner loop");
    declare_parameter("inlier_threshold", 85, "Threshold for the number of inliers to consider a model valid");
    declare_parameter("hough_radius_bins", 50, "Number of radius bins for the circle Hough space");
    declare_parameter("hough_center_bins", 50, "Number of center bins (in each dimension) for the circle Hough space");
    declare_parameter("ransac_distance_threshold", 0.001, "Distance threshold for RANSAC (how close a point must be to the model to be considered an inlier)");
    declare_parameter("ransac_max_iterations", 1000, "Maximum number of iterations for the RANSAC algorithm");
    // Declare parameters for circle filtering
    declare_parameter("circle_min_cluster_size", 20, "Minimum size for a cluster of circle inliers");
    declare_parameter("circle_max_clusters", 2, "Maximum number of allowed clusters for circles");
    declare_parameter("circle_height_tolerance", 0.025, "Tolerance for height difference between circle clusters");
    declare_parameter("circle_curvature_threshold", 0.0015, "Threshold for point curvature in circle fitting");
    declare_parameter("circle_radius_tolerance", 0.020, "Tolerance for difference between point RSD min value and circle radius");
    declare_parameter("circle_normal_angle_threshold", 0.2, "Threshold for angle between point normal and circle radial vector");
    declare_parameter("circle_cluster_tolerance", 0.025, "The maximum distance between two points to be considered in the same cluster");
    // Declare new parameters for line filtering
    declare_parameter("line_min_cluster_size", 20, "Minimum number of points for a valid cluster");
    declare_parameter("line_max_clusters", 1, "Maximum number of allowed clusters for lines");
    declare_parameter("line_curvature_threshold", 0.0011, "Threshold for point curvature in line fitting");
    declare_parameter("line_cluster_tolerance", 0.025, "The maximum distance between two points to be considered in the same cluster for lines");
    declare_parameter("line_rho_threshold", 0.01, "Tolerance for rho. Used for clustering similar candidate line models before voting.");
    declare_parameter("line_theta_threshold", 0.1, "Tolerance for theta. Used for clustering similar candidate line models before voting.");
    // Legacy...remove these later
    declare_parameter("shape_fitting_max_iterations", 1000, "Maximum iterations for shape fitting RANSAC");
    declare_parameter("shape_fitting_distance_threshold", 0.01, "Distance threshold for shape fitting (in meters)");
    declare_parameter("shape_fitting_min_radius", 0.01, "Minimum radius for cylinder fitting (in meters)");
    declare_parameter("shape_fitting_max_radius", 0.1, "Maximum radius for cylinder fitting (in meters)");
    declare_parameter("shape_fitting_normal_distance_weight", 0.1, "Normal distance weight for cylinder fitting");
    declare_parameter("shape_fitting_normal_search_radius", 0.05, "Search radius for normal estimation in shape fitting (in meters)");

    // Output directory for point clouds. Useful for debugging
    // ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:=/home/ubuntu/Downloads/my_debug_cloud.pcd -p frame_id:=base_link -p interval:=1.0
    declare_parameter("output_directory", "/tmp/", "Directory to save output PCD files");
    declare_parameter("debug_pcd_filename", "debug_cloud.pcd", "Filename for debug PCD output");

    // Get parameter values
    point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
    rgb_image_topic = this->get_parameter("rgb_image_topic").as_string();
    target_frame = this->get_parameter("target_frame").as_string();

    // Get parameters for cropping
    enable_cropping = this->get_parameter("enable_cropping").as_bool();
    crop_min_x = this->get_parameter("crop_min_x").as_double();
    crop_max_x = this->get_parameter("crop_max_x").as_double();
    crop_min_y = this->get_parameter("crop_min_y").as_double();
    crop_max_y = this->get_parameter("crop_max_y").as_double();
    crop_min_z = this->get_parameter("crop_min_z").as_double();
    crop_max_z = this->get_parameter("crop_max_z").as_double();

    // Get parameters for plane and object segmentation
    max_iterations = this->get_parameter("max_iterations").as_int();
    distance_threshold = this->get_parameter("distance_threshold").as_double();
    z_tolerance = this->get_parameter("z_tolerance").as_double();
    angle_tolerance = this->get_parameter("angle_tolerance").as_double();
    plane_segmentation_threshold = this->get_parameter("plane_segmentation_threshold").as_double();
    min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size = this->get_parameter("max_cluster_size").as_int();
    cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    normal_estimation_k = this->get_parameter("normal_estimation_k").as_int();
    w_inliers = this->get_parameter("w_inliers").as_double();
    w_size = this->get_parameter("w_size").as_double();
    w_distance = this->get_parameter("w_distance").as_double();
    w_orientation = this->get_parameter("w_orientation").as_double();
    max_plane_segmentation_iterations = this->get_parameter("max_plane_segmentation_iterations").as_int();
    plane_segmentation_distance_threshold = this->get_parameter("plane_segmentation_distance_threshold").as_double();

    // Get parameters for creating the support plane
    support_surface_name = this->get_parameter("support_surface_name").as_string();
    min_surface_thickness = this->get_parameter("min_surface_thickness").as_double();

    // Get parameters for normal, curvature, and RSD estimation
    k_neighbors = this->get_parameter("k_neighbors").as_int();
    max_plane_error = this->get_parameter("max_plane_error").as_double();
    max_iterations_normals = this->get_parameter("max_iterations_normals").as_int();
    min_boundary_neighbors = this->get_parameter("min_boundary_neighbors").as_int();
    rsd_radius = this->get_parameter("rsd_radius").as_double();

    // Get clustering parameter values
    nearest_neighbors = this->get_parameter("nearest_neighbors").as_int();
    smoothness_threshold = this->get_parameter("smoothness_threshold").as_double();
    curvature_threshold = this->get_parameter("curvature_threshold").as_double();

    // Get object segmentation values
    num_iterations = this->get_parameter("num_iterations").as_int();
    inlier_threshold = this->get_parameter("inlier_threshold").as_int();
    hough_radius_bins = this->get_parameter("hough_radius_bins").as_int();
    hough_center_bins = this->get_parameter("hough_center_bins").as_int();
    ransac_distance_threshold = this->get_parameter("ransac_distance_threshold").as_double();
    ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();
    // Get parameter values for circle filtering
    circle_min_cluster_size = this->get_parameter("circle_min_cluster_size").as_int();
    circle_max_clusters = this->get_parameter("circle_max_clusters").as_int();
    circle_height_tolerance = this->get_parameter("circle_height_tolerance").as_double();
    circle_curvature_threshold = this->get_parameter("circle_curvature_threshold").as_double();
    circle_radius_tolerance = this->get_parameter("circle_radius_tolerance").as_double();
    circle_normal_angle_threshold = this->get_parameter("circle_normal_angle_threshold").as_double();
    circle_cluster_tolerance = this->get_parameter("circle_cluster_tolerance").as_double();
    // Get parameter values for line filtering
    line_min_cluster_size = this->get_parameter("line_min_cluster_size").as_int();
    line_max_clusters = this->get_parameter("line_max_clusters").as_int();
    line_curvature_threshold = this->get_parameter("line_curvature_threshold").as_double();
    line_cluster_tolerance = this->get_parameter("line_cluster_tolerance").as_double();
    line_rho_threshold = this->get_parameter("line_rho_threshold").as_double();
    line_theta_threshold = this->get_parameter("line_theta_threshold").as_double();

    // Legacy...remove later Get shape fitting parameter values
    shape_fitting_max_iterations = this->get_parameter("shape_fitting_max_iterations").as_int();
    shape_fitting_distance_threshold = this->get_parameter("shape_fitting_distance_threshold").as_double();
    shape_fitting_min_radius = this->get_parameter("shape_fitting_min_radius").as_double();
    shape_fitting_max_radius = this->get_parameter("shape_fitting_max_radius").as_double();
    shape_fitting_normal_distance_weight = this->get_parameter("shape_fitting_normal_distance_weight").as_double();
    shape_fitting_normal_search_radius = this->get_parameter("shape_fitting_normal_search_radius").as_double();

    // Output directory for point cloud files
    output_directory = this->get_parameter("output_directory").as_string();
    debug_pcd_filename = this->get_parameter("debug_pcd_filename").as_string();
  }

  void createSubscribers() {
    // Create subscriber for point cloud data
    point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic,
      10,  // QoS depth
      std::bind(&GetPlanningSceneServer::pointCloudCallback, this, std::placeholders::_1)
    );

    // Create subscriber for RGB image data
    rgb_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      rgb_image_topic,
      10,  // QoS depth
      std::bind(&GetPlanningSceneServer::rgbImageCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud topic: %s", point_cloud_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to RGB image topic: %s", rgb_image_topic.c_str());
  }

  void createService() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    service = this->create_service<mycobot_interfaces::srv::GetPlanningScene>(
      "get_planning_scene_mycobot",
     std::bind(&GetPlanningSceneServer::handleService, this, std::placeholders::_1, std::placeholders::_2),
      qos
    );

    RCLCPP_INFO(this->get_logger(), "Get planning scene service created and ready to serve requests.");
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg != nullptr && !msg->data.empty()) {
      latest_point_cloud = msg;
    }
  }

  void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg != nullptr && !msg->data.empty()) {
      latest_rgb_image = msg;
    }
  }

  sensor_msgs::msg::PointCloud2::SharedPtr transformPointCloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
      const std::string& target_frame) {
    RCLCPP_INFO(this->get_logger(), "Transforming point cloud from %s to %s",
      cloud_msg->header.frame_id.c_str(), target_frame.c_str());

    if (cloud_msg->header.frame_id == target_frame) {
      RCLCPP_INFO(this->get_logger(), "Point cloud is already in the target frame");
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Look up the transformation from the cloud's frame to the target frame
      transform_stamped = tf_buffer_->lookupTransform(
        target_frame, cloud_msg->header.frame_id,
        tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform point cloud: %s", ex.what());
      return nullptr;
    }

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // Create transformation matrix from transform_stamped
    Eigen::Affine3d transform_eigen;
    transform_eigen = tf2::transformToEigen(transform_stamped);

    // Transform the PCL cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform_eigen);

    if (enable_cropping) {
      RCLCPP_INFO(this->get_logger(), "Cropping is enabled. Applying crop box filter.");

      // Crop the transformed cloud
      pcl::CropBox<pcl::PointXYZRGB> crop_box;
      crop_box.setInputCloud(transformed_cloud);

      crop_box.setMin(Eigen::Vector4f(crop_min_x, crop_min_y, crop_min_z, 1.0));
      crop_box.setMax(Eigen::Vector4f(crop_max_x, crop_max_y, crop_max_z, 1.0));

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      crop_box.filter(*cropped_cloud);

      transformed_cloud = cropped_cloud;
      RCLCPP_INFO(this->get_logger(), "Point cloud cropped. New size: %zu points", transformed_cloud->size());
    } else {
      RCLCPP_INFO(this->get_logger(), "Cropping is disabled. Using full transformed point cloud.");
    }

    // Convert back to ROS PointCloud2
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_out(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud, *cloud_out);

    // Update the frame_id and timestamp of the transformed cloud
    cloud_out->header.frame_id = target_frame;
    cloud_out->header.stamp = this->now();

    RCLCPP_INFO(this->get_logger(), "Point cloud transformed successfully");
    return cloud_out;
  }

  // Conversion from PointCloud2 to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    RCLCPP_INFO(this->get_logger(), "Converting PointCloud2 to PCL PointCloud");

    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    try {
        if (!cloud_msg) {
            throw std::runtime_error("Input PointCloud2 message is null");
        }
        if (cloud_msg->data.empty()) {
            throw std::runtime_error("Input PointCloud2 message has no data");
        }

        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

        if (pcl_cloud->empty()) {
            throw std::runtime_error("Resulting PCL cloud is empty after conversion");
        }

        RCLCPP_INFO(this->get_logger(), "PointCloud2 successfully converted to PCL PointCloud with %zu points",
            pcl_cloud->size());
    } catch (const pcl::PCLException& e) {
        RCLCPP_ERROR(this->get_logger(), "PCL error in convertToPCL: %s", e.what());
        return nullptr;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in convertToPCL: %s", e.what());
        return nullptr;
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error occurred in convertToPCL");
        return nullptr;
    }

    return pcl_cloud;
  }

  moveit_msgs::msg::CollisionObject createSupportSurfaceObject(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_cloud,
      pcl::ModelCoefficients::Ptr plane_coefficients,
      const std::string& frame_id) {
    RCLCPP_INFO(this->get_logger(), "Creating support surface object");

    moveit_msgs::msg::CollisionObject support_surface;

    try {
      // Validate input parameters
      if (!plane_coefficients || plane_coefficients->values.size() != 4) {
        throw std::invalid_argument("Invalid plane coefficients");
      }
      if (frame_id.empty()) {
        throw std::invalid_argument("Empty frame_id");
      }
      if (!plane_cloud || plane_cloud->empty()) {
        throw std::invalid_argument("Invalid or empty plane point cloud");
      }

      // Calculate bounding box of the plane cloud
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);

      // If cropping is enabled, limit the bounding box to the crop box limits
      if (enable_cropping) {
        min_pt[0] = std::max(min_pt[0], static_cast<float>(crop_min_x));
        min_pt[1] = std::max(min_pt[1], static_cast<float>(crop_min_y));
        min_pt[2] = std::max(min_pt[2], static_cast<float>(crop_min_z));
        max_pt[0] = std::min(max_pt[0], static_cast<float>(crop_max_x));
        max_pt[1] = std::min(max_pt[1], static_cast<float>(crop_max_y));
        max_pt[2] = std::min(max_pt[2], static_cast<float>(crop_max_z));
      }

      // Calculate centroid
      Eigen::Vector4f centroid;
      centroid = (min_pt + max_pt) / 2.0f;

      // Create a box primitive
      shape_msgs::msg::SolidPrimitive box_primitive;
      box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      box_primitive.dimensions.resize(3);
      box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = max_pt[0] - min_pt[0];
      box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = max_pt[1] - min_pt[1];
      box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = max_pt[2] - min_pt[2];

      // Ensure a minimum thickness
      if (box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] < min_surface_thickness) {
        box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = min_surface_thickness;
      }

      // Calculate an appropriate pose for the box based on the plane's normal vector
      geometry_msgs::msg::Pose box_pose;
      Eigen::Vector3d normal(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);
      normal.normalize();

      // Calculate rotation to align the plane normal with the z-axis
      Eigen::Quaterniond rotation;
      rotation.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal);

      // Set the pose
      box_pose.position.x = centroid[0];
      box_pose.position.y = centroid[1];
      box_pose.position.z = centroid[2];
      box_pose.orientation.x = rotation.x();
      box_pose.orientation.y = rotation.y();
      box_pose.orientation.z = rotation.z();
      box_pose.orientation.w = rotation.w();

      RCLCPP_INFO(this->get_logger(), "Support surface box dimensions: [%.4f, %.4f, %.4f]",
        box_primitive.dimensions[0], box_primitive.dimensions[1], box_primitive.dimensions[2]);
      RCLCPP_INFO(this->get_logger(), "Support surface orientation: [%.4f, %.4f, %.4f, %.4f]",
        box_pose.orientation.x, box_pose.orientation.y,
        box_pose.orientation.z, box_pose.orientation.w);

      // Set up the CollisionObject
      support_surface.header.frame_id = frame_id;
      support_surface.header.stamp = this->now();
      support_surface.id = support_surface_name;
      support_surface.primitives.push_back(box_primitive);
      support_surface.primitive_poses.push_back(box_pose);
      support_surface.operation = moveit_msgs::msg::CollisionObject::ADD;

      RCLCPP_INFO(this->get_logger(), "Support surface object created successfully as a box");
      RCLCPP_INFO(this->get_logger(), "Support surface normal: [%.4f, %.4f, %.4f]",
        normal.x(), normal.y(), normal.z());
      RCLCPP_INFO(this->get_logger(), "Support surface position: [%.4f, %.4f, %.4f]",
        box_pose.position.x, box_pose.position.y, box_pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Support surface orientation: [%.4f, %.4f, %.4f, %.4f]",
        box_pose.orientation.x, box_pose.orientation.y,
        box_pose.orientation.z, box_pose.orientation.w);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error creating support surface object: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error occurred while creating support surface object");
    }
    return support_surface;
  }

  moveit_msgs::msg::CollisionObject fitShapeToCluster(
      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cluster,
      const std::string& frame_id,
      int index) {
    RCLCPP_INFO(this->get_logger(), "Fitting shape to cluster %d", index);

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "object_" + std::to_string(index);

    if (cluster->empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty cluster. Skipping shape fitting.");
      return collision_object;
    }

    // Structure to store fitting results
    struct FitResult {
      std::string shape_type;
      std::shared_ptr<pcl::ModelCoefficients> coefficients;
      std::shared_ptr<pcl::PointIndices> inliers;
      double fitness_score;
    };

    std::vector<FitResult> fit_results;

    // Attempt to fit multiple primitive shapes using RANSAC
    // Box fitting
    {
      auto seg = std::make_shared<pcl::SACSegmentation<pcl::PointXYZRGB>>();
      seg->setOptimizeCoefficients(true);
      seg->setModelType(pcl::SACMODEL_PARALLEL_PLANE);
      seg->setMethodType(pcl::SAC_RANSAC);
      seg->setMaxIterations(shape_fitting_max_iterations);
      seg->setDistanceThreshold(shape_fitting_distance_threshold);

      FitResult box_fit;
      box_fit.shape_type = "box";
      box_fit.coefficients = std::make_shared<pcl::ModelCoefficients>();
      box_fit.inliers = std::make_shared<pcl::PointIndices>();

      seg->setInputCloud(cluster);
      seg->segment(*(box_fit.inliers), *(box_fit.coefficients));

      if (box_fit.inliers->indices.size() > 0) {
        auto box_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::copyPointCloud(*cluster, box_fit.inliers->indices, *box_cloud);
        box_fit.fitness_score = static_cast<double>(box_fit.inliers->indices.size()) / cluster->size();
        fit_results.push_back(box_fit);
      }
    }

    // Cylinder fitting
    {
      auto seg = std::make_shared<pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal>>();
      seg->setOptimizeCoefficients(true);
      seg->setModelType(pcl::SACMODEL_CYLINDER);
      seg->setMethodType(pcl::SAC_RANSAC);
      seg->setMaxIterations(shape_fitting_max_iterations);
      seg->setDistanceThreshold(shape_fitting_distance_threshold);
      seg->setRadiusLimits(shape_fitting_min_radius, shape_fitting_max_radius);
      seg->setNormalDistanceWeight(shape_fitting_normal_distance_weight);

      auto ne = std::make_shared<pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal>>();
      auto cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
      auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
      ne->setSearchMethod(tree);
      ne->setInputCloud(cluster);
      ne->setRadiusSearch(shape_fitting_normal_search_radius);
      ne->compute(*cloud_normals);

      seg->setInputCloud(cluster);
      seg->setInputNormals(cloud_normals);

      FitResult cylinder_fit;
      cylinder_fit.shape_type = "cylinder";
      cylinder_fit.coefficients = std::make_shared<pcl::ModelCoefficients>();
      cylinder_fit.inliers = std::make_shared<pcl::PointIndices>();

      seg->segment(*(cylinder_fit.inliers), *(cylinder_fit.coefficients));

      if (cylinder_fit.inliers->indices.size() > 0) {
        cylinder_fit.fitness_score = static_cast<double>(cylinder_fit.inliers->indices.size()) / cluster->size();
        fit_results.push_back(cylinder_fit);
      }
    }

    // Compare fitted shapes and select the best fit
    FitResult best_fit;
    double best_score = 0.0;
    for (const auto& fit : fit_results) {
      if (fit.fitness_score > best_score) {
        best_score = fit.fitness_score;
        best_fit = fit;
      }
    }

    if (best_fit.shape_type.empty()) {
      RCLCPP_WARN(this->get_logger(), "No shape could be fitted to the cluster.");
      return collision_object;
    }

    // Create CollisionObject for the best-fitting shape
    shape_msgs::msg::SolidPrimitive primitive;
    geometry_msgs::msg::Pose pose;

    if (best_fit.shape_type == "box") {
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions.resize(3);
      // Calculate box dimensions from point cloud
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = max_pt[0] - min_pt[0];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = max_pt[1] - min_pt[1];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = max_pt[2] - min_pt[2];
      pose.position.x = (min_pt[0] + max_pt[0]) / 2;
      pose.position.y = (min_pt[1] + max_pt[1]) / 2;
      pose.position.z = (min_pt[2] + max_pt[2]) / 2;
    } else if (best_fit.shape_type == "cylinder") {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions.resize(2);
      Eigen::Vector3f axis(best_fit.coefficients->values[3], best_fit.coefficients->values[4], best_fit.coefficients->values[5]);
      Eigen::Vector3f center(best_fit.coefficients->values[0], best_fit.coefficients->values[1], best_fit.coefficients->values[2]);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = best_fit.coefficients->values[6];

      // Calculate cylinder height
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);
      Eigen::Vector3f min_vec = min_pt.head<3>();
      Eigen::Vector3f max_vec = max_pt.head<3>();
      Eigen::Vector3f diff = max_vec - min_vec;
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = diff.dot(axis.normalized());

      Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), axis);
      pose.position.x = center[0];
      pose.position.y = center[1];
      pose.position.z = center[2];
      pose.orientation.x = quat.x();
      pose.orientation.y = quat.y();
      pose.orientation.z = quat.z();
      pose.orientation.w = quat.w();
    }

    collision_object.id = best_fit.shape_type + "_" + std::to_string(index);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    RCLCPP_INFO(this->get_logger(), "Fitted %s to cluster %d with score %.2f",
                best_fit.shape_type.c_str(), index, best_score);

    return collision_object;
  }

  std::string identifyTargetObject(
      const std::vector<moveit_msgs::msg::CollisionObject>& objects,
      const std::string& target_shape,
      const std::vector<double>& target_dimensions) {

    double best_score = 0.0;
    std::string best_match_id;

    for (const auto& object : objects) {
      if (object.primitives.empty()) continue;

      const auto& primitive = object.primitives[0];
      double shape_score = 0.0;
      double dimension_score = 0.0;

      // Compare shape types
      if ((target_shape == "cylinder" && primitive.type == shape_msgs::msg::SolidPrimitive::CYLINDER) ||
          (target_shape == "box" && primitive.type == shape_msgs::msg::SolidPrimitive::BOX)) {
        shape_score = 1.0;
      } else {
        continue; // Skip to next object if shape doesn't match
      }

      // Compare dimensions
      std::vector<double> object_dimensions;
      switch (primitive.type) {
        case shape_msgs::msg::SolidPrimitive::CYLINDER:
          object_dimensions = {primitive.dimensions[primitive.CYLINDER_HEIGHT],
                               primitive.dimensions[primitive.CYLINDER_RADIUS]};
          break;
        case shape_msgs::msg::SolidPrimitive::BOX:
          object_dimensions = {primitive.dimensions[primitive.BOX_X],
                               primitive.dimensions[primitive.BOX_Y],
                               primitive.dimensions[primitive.BOX_Z]};
          break;
        default:
          continue; // Skip to next object if shape is neither cylinder nor box
      }

      // Calculate dimension similarity score
      if (object_dimensions.size() == target_dimensions.size()) {
        double total_diff = 0.0;
        for (size_t i = 0; i < object_dimensions.size(); ++i) {
          double diff = std::abs(object_dimensions[i] - target_dimensions[i]);
          total_diff += diff / target_dimensions[i]; // Normalize the difference
        }
        dimension_score = 1.0 - (total_diff / object_dimensions.size()); // Average normalized similarity
        dimension_score = std::max(0.0, dimension_score); // Ensure non-negative score
      }

      // Calculate overall similarity score
      double similarity_score = 0.7 * shape_score + 0.3 * dimension_score;

      // Update best match if this object has a higher similarity score
      if (similarity_score > best_score) {
        best_score = similarity_score;
        best_match_id = object.id;
      }
    }

    if (!best_match_id.empty()) {
      RCLCPP_INFO(this->get_logger(),
                  "Best matching object found: %s with similarity score: %.2f",
                  best_match_id.c_str(), best_score);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "No matching object found for the target shape and dimensions");
    }

    return best_match_id;
  }

  moveit_msgs::msg::PlanningSceneWorld assemblePlanningSceneWorld(
    const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects) {

    RCLCPP_INFO(this->get_logger(), "Assembling PlanningSceneWorld");

    moveit_msgs::msg::PlanningSceneWorld planning_scene_world;

    for (const auto& object : collision_objects) {
      planning_scene_world.collision_objects.push_back(object);

      if (object.header.frame_id != target_frame) {
        RCLCPP_WARN(this->get_logger(),
                    "CollisionObject '%s' is in frame '%s', not in target frame '%s'. "
                    "This may cause issues in planning.",
                    object.id.c_str(), object.header.frame_id.c_str(), target_frame.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Assembled PlanningSceneWorld successfully with %zu collision objects",
                planning_scene_world.collision_objects.size());

    return planning_scene_world;
  }

  // Used for debugging to see the point cloud at interim steps
  void savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename) {
    std::string full_path = output_directory + filename;
    if (pcl::io::savePCDFileBinary(full_path, *cloud) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save %s", full_path.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Saved %s with %zu points.", full_path.c_str(), cloud->size());
    }
  }

  /***********************************************************************************
   *                                                                                 *
   *                               Main Method                                       *
   *                                                                                 *
   **********************************************************************************/
  void handleService(
      const std::shared_ptr<mycobot_interfaces::srv::GetPlanningScene::Request> request,
      std::shared_ptr<mycobot_interfaces::srv::GetPlanningScene::Response> response) {

    /****************************************************
     *                                                  *
     * 1. Initialize response success flag to false     *
     *                                                  *
     ***************************************************/
    response->success = false;

    /****************************************************
     *                                                  *
     * 2. Point cloud and RGB image data available?     *
     *                                                  *
     ***************************************************/
    if (!latest_point_cloud || !latest_rgb_image) {
      RCLCPP_ERROR(this->get_logger(), "Point cloud or RGB image data not available");
      return;
    }

    /***********************************************************************************
     *                                                                                 *
     * 3. Validate input parameters & prepare point cloud:                             *
     *    - Check if target_shape and target_dimensions are not empty                  *
     *    - Verify that target_shape is one of the valid shapes (cylinder, box)        *
     *    - Store the original point cloud frame for reference                         *
     *    - Transform the point cloud to the target frame for consistent processing    *
     *    - Apply optional clipping (cropping) to the point cloud based on parameters  *
     *    - If any validation or transformation fails, log an error and return early   *
     *                                                                                 *
     **********************************************************************************/
    if (request->target_shape.empty() || request->target_dimensions.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid input parameters: target_shape or target_dimensions is empty");
      return;
    }

    const std::vector<std::string> valid_shapes = {"cylinder", "box"};
    if (std::find(valid_shapes.begin(), valid_shapes.end(), request->target_shape) == valid_shapes.end()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid target_shape: %s", request->target_shape.c_str());
      return;
    }

    // Store the original point cloud frame
    std::string original_cloud_frame = latest_point_cloud->header.frame_id;

    // Transform the point cloud to the target frame for processing
    auto transformed_cloud = transformPointCloud(latest_point_cloud, target_frame);
    if (!transformed_cloud) {
      RCLCPP_ERROR(this->get_logger(), "Failed to transform point cloud to target frame");
      return;
    }

    /***********************************************************************************
     *                                                                                 *
     * 4. Convert PointCloud2 to PCL point cloud                                       *
     *    - Check if conversion was successful and resulting cloud is not empty        *
     *    - If conversion fails, log an error and return early                         *
     *                                                                                 *
     **********************************************************************************/
    auto pcl_cloud = convertToPCL(transformed_cloud);
    if (!pcl_cloud) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert PointCloud2 to PCL format");
      return;
    }
    //  Used for debugging and visualization
    savePointCloudToPCD(pcl_cloud, "4_convertToPCL_" + debug_pcd_filename);

    /***********************************************************************************
     *                                                                                 *
     * 5. Segment the support plane and objects from a given point cloud.              *
     *                                                                                 *
     **********************************************************************************/
    auto [support_plane_cloud, objects_cloud, plane_coefficients] = segmentPlaneAndObjects(
      pcl_cloud,
      enable_cropping,
      crop_min_x, crop_max_x,
      crop_min_y, crop_max_y,
      crop_min_z, crop_max_z,
      max_plane_segmentation_iterations,
      plane_segmentation_distance_threshold,
      z_tolerance,
      angle_tolerance,
      min_cluster_size,
      max_cluster_size,
      cluster_tolerance,
      normal_estimation_k,
      plane_segmentation_threshold,
      w_inliers,
      w_size,
      w_distance,
      w_orientation
    );

    if (!support_plane_cloud || !objects_cloud || !plane_coefficients) {
      RCLCPP_ERROR(this->get_logger(), "Plane and object segmentation failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Plane and object segmentation successful");
    RCLCPP_INFO(this->get_logger(), "Support plane cloud size: %zu", support_plane_cloud->size());
    RCLCPP_INFO(this->get_logger(), "Plane coefficients: [%.3f, %.3f, %.3f, %.3f]",
      plane_coefficients->values[0], plane_coefficients->values[1],
      plane_coefficients->values[2], plane_coefficients->values[3]);
    RCLCPP_INFO(this->get_logger(), "Objects cloud size: %zu", objects_cloud->size());

    // For debugging
    savePointCloudToPCD(support_plane_cloud, "5_support_plane_" + debug_pcd_filename);
    savePointCloudToPCD(objects_cloud, "5_objects_cloud_" + debug_pcd_filename);

    /***********************************************************************************
     *                                                                                 *
     * 6. Create CollisionObject for support surface                                   *
     *    - Check if support surface object creation was successful                    *
     *    - If creation fails, log a warning                                           *
     *                                                                                 *
     **********************************************************************************/
    moveit_msgs::msg::CollisionObject support_surface = createSupportSurfaceObject(
      support_plane_cloud,
      plane_coefficients,
      target_frame
    );

    if (support_surface.id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Support surface collision object creation failed or resulted in an invalid object");
    } else {
      RCLCPP_INFO(this->get_logger(), "Adding support surface collision object to the planning scene");
      // Add the support surface to the planning scene
      response->scene_world.collision_objects.push_back(support_surface);
      // Set the support_surface_id in the response
      response->support_surface_id = support_surface.id;
    }

    /***********************************************************************************
     *                                                                                 *
     * 7. Estimate normal vectors, curvature values, and RSD values for                *
     *      each point in the point cloud.                                             *
     *                                                                                 *
     **********************************************************************************/
    cloud_with_features = estimateNormalsCurvatureAndRSD(
      objects_cloud,
      k_neighbors,
      max_plane_error,
      max_iterations_normals,
      min_boundary_neighbors,
      rsd_radius
    );

    if (!cloud_with_features || cloud_with_features->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to estimate normals, curvature, and RSD");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully estimated normals, curvature, and RSD for %zu points",
      cloud_with_features->size());

    /***********************************************************************************
     *                                                                                 *
     * 8. Extract clusters                                                             *
     *    - Extract point cloud clusters using region growing (nearest neighbors).     *
     *                                                                                 *
     **********************************************************************************/
    std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr> clusters = extractClusters(
      cloud_with_features,
      min_cluster_size,
      max_cluster_size,
      smoothness_threshold,
      curvature_threshold,
      nearest_neighbors
    );

    if (clusters.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Node '%s' failed to extract any clusters from the point cloud", this->get_name());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Node '%s' successfully extracted %zu clusters from the point cloud", this->get_name(), clusters.size());

    /***********************************************************************************
     *                                                                                 *
     * 9. Get collision objects                                                        *
     *    - Segment the point cloud clusters into distinct collision objects.          *
     *                                                                                 *
     **********************************************************************************/
    std::vector<moveit_msgs::msg::CollisionObject> segmented_objects = segmentObjects(
      clusters,
      num_iterations,
      target_frame,
      inlier_threshold,
      hough_radius_bins,
      hough_center_bins,
      ransac_distance_threshold,
      ransac_max_iterations,
      circle_min_cluster_size,
      circle_max_clusters,
      circle_height_tolerance,
      circle_curvature_threshold,
      circle_radius_tolerance,
      circle_normal_angle_threshold,
      circle_cluster_tolerance,
      line_min_cluster_size,
      line_max_clusters,
      line_curvature_threshold,
      line_cluster_tolerance,
      line_rho_threshold,
      line_theta_threshold
    );

    RCLCPP_INFO(this->get_logger(), "Segmented %zu objects from the point cloud clusters", segmented_objects.size());

    // Add segmented objects to the planning scene
    for (const auto& object : segmented_objects) {
      response->scene_world.collision_objects.push_back(object);
    }

    /***********************************************************************************
     *                                                                                 *
     * 10. Identify target object                                                      *
     *     - Check if a target object was successfully identified                      *
     *     - If no target found, log a warning                                         *
     *                                                                                 *
     **********************************************************************************/
    std::string target_object_id = identifyTargetObject(
      response->scene_world.collision_objects,
      request->target_shape,
      request->target_dimensions);

    if (!target_object_id.empty()) {
      response->target_object_id = target_object_id;
    }

    /***********************************************************************************
     *                                                                                 *
     * 11. Assemble PlanningSceneWorld                                                 *
     *     - Check if assembly was successful                                          *
     *     - If assembly fails, log an error and return early                          *
     *                                                                                 *
     **********************************************************************************/
    try {
      response->scene_world = assemblePlanningSceneWorld(response->scene_world.collision_objects);

      if (response->scene_world.collision_objects.empty()) {
        RCLCPP_ERROR(this->get_logger(), "PlanningSceneWorld assembly resulted in no collision objects");
        return;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to assemble PlanningSceneWorld: %s", e.what());
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error occurred while assembling PlanningSceneWorld");
      return;
    }

    /***********************************************************************************
     *                                                                                 *
     * 12. Fill the rest of the response:                                              *
     *     - Set full_cloud, rgb_image, and target_object_id                           *
     *     - Set success flag to true if all critical steps were successful            *
     *                                                                                 *
     **********************************************************************************/
    response->full_cloud = *latest_point_cloud;  // Use the transformed cloud
    response->rgb_image = *latest_rgb_image;
    response->target_object_id = target_object_id;

    // Helpful logging
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Success: %s", response->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Target object ID: %s", response->target_object_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Support surface ID: %s", response->support_surface_id.c_str());

    // Point cloud information
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Full cloud frame ID: %s", response->full_cloud.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Full cloud size: %d x %d",
      response->full_cloud.width, response->full_cloud.height);

    // RGB image information
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "RGB image frame ID: %s", response->rgb_image.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "RGB image size: %d x %d",
      response->rgb_image.width, response->rgb_image.height);

    // Collision objects information
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Number of collision objects: %zu",
      response->scene_world.collision_objects.size());
    for (const auto& obj : response->scene_world.collision_objects) {
      if (!obj.primitives.empty()) {
        const auto& primitive = obj.primitives[0];
        const auto& pose = obj.primitive_poses[0];
        std::string type_str;
        std::string dimensions_str;

        switch(primitive.type) {
          case shape_msgs::msg::SolidPrimitive::BOX:
            type_str = "BOX";
            dimensions_str = "x=" + std::to_string(primitive.dimensions[0]) + ", " +
                             "y=" + std::to_string(primitive.dimensions[1]) + ", " +
                             "z=" + std::to_string(primitive.dimensions[2]);
            break;
          case shape_msgs::msg::SolidPrimitive::CYLINDER:
            type_str = "CYLINDER";
            dimensions_str = "height=" + std::to_string(primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT]) + ", " +
                             "radius=" + std::to_string(primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS]);
            break;
          default:
            continue;  // Skip other primitive types
        }

        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "Collision Object: ID=%s, Frame=%s, Type=%s",
          obj.id.c_str(), obj.header.frame_id.c_str(), type_str.c_str());
        RCLCPP_INFO(this->get_logger(), "  Position: x=%.4f, y=%.4f, z=%.4f (meters)",
          pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
          pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "  Dimensions: %s", dimensions_str.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_WARN(this->get_logger(), "Collision Object: ID=%s has no primitives",
          obj.id.c_str());
      }
    }

    // Additional processing information
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Original point cloud frame: %s", latest_point_cloud->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Target frame used for processing: %s", target_frame.c_str());

    // Check if all critical steps were successful
    if (!response->scene_world.collision_objects.empty() &&
        !response->full_cloud.data.empty() &&
        !response->rgb_image.data.empty() &&
        !response->target_object_id.empty() &&
        !response->support_surface_id.empty()) {
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "===== Service response filled successfully ===== ");
    } else {
      response->success = false;
      RCLCPP_WARN(this->get_logger(), "===== Service response incomplete or invalid. Here is an explanation: =====");
      if (response->scene_world.collision_objects.empty()) {
        RCLCPP_WARN(this->get_logger(), "  No collision objects in the scene");
      }
      if (response->full_cloud.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "  Empty point cloud data");
      }
      if (response->rgb_image.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "  Empty RGB image data");
      }
      if (response->target_object_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "  The target object was not found in the input point cloud");
      }
      if (response->support_surface_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "  The support surface ID is empty");
      }
    }
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Service response logging completed!");
  }
};

int main(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<GetPlanningSceneServer>(options)
    ;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("get_planning_scene_server"), "Caught exception: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("get_planning_scene_server"), "Caught unknown exception");
    return 1;
  }

  return 0;
}
