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
 *     success (bool): Indicates if the operation was successful
 *
 * @author Addison Sears-Collins
 * @date September 11, 2024
 */
 
 #include "hello_mtc_with_perception/plane_segmentation.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <mycobot_interfaces/srv/get_planning_scene.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

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
  
  // Legacy...remove later Parameters for plane segmentation
  int max_plane_segmentation_iterations;
  double plane_segmentation_distance_threshold;
  
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
    declare_parameter("distance_threshold", 0.02, "Distance threshold for RANSAC");
    declare_parameter("z_tolerance", 0.05, "Tolerance for z-coordinate of the support plane");
    declare_parameter("angle_tolerance", 0.996194698, "Angle tolerance for surface normals"); 
    declare_parameter("plane_segmentation_threshold", 0.02, "Threshold for plane segmentation");
    declare_parameter("min_cluster_size", 100, "Minimum size of a cluster");
    declare_parameter("max_cluster_size", 25000, "Maximum size of a cluster");
    declare_parameter("cluster_tolerance", 0.02, "Tolerance for Euclidean clustering");
    declare_parameter("normal_estimation_k", 30, "Number of neighbors for normal estimation");
    declare_parameter("w_inliers", 1.0, "Weight for inlier score in plane model selection");
    declare_parameter("w_size", 1.0, "Weight for size score in plane model selection");
    declare_parameter("w_distance", 1.0, "Weight for distance score in plane model selection");
    declare_parameter("w_orientation", 1.0, "Weight for orientation score in plane model selection");
  
    // Legacy...remove these later
    declare_parameter("max_plane_segmentation_iterations", 1000, "Maximum iterations for plane segmentation RANSAC");
    declare_parameter("plane_segmentation_distance_threshold", 0.01, "Distance threshold for plane segmentation (in meters)");  

   
    // Legacy...remove these later
    declare_parameter("shape_fitting_max_iterations", 1000, "Maximum iterations for shape fitting RANSAC");
    declare_parameter("shape_fitting_distance_threshold", 0.01, "Distance threshold for shape fitting (in meters)");
    declare_parameter("shape_fitting_min_radius", 0.01, "Minimum radius for cylinder fitting (in meters)");
    declare_parameter("shape_fitting_max_radius", 0.1, "Maximum radius for cylinder fitting (in meters)");
    declare_parameter("shape_fitting_normal_distance_weight", 0.1, "Normal distance weight for cylinder fitting");
    declare_parameter("shape_fitting_normal_search_radius", 0.05, "Search radius for normal estimation in shape fitting (in meters)");
    
    // Output directory for point clouds. Useful for debugging
    // ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:=/home/ubuntu/Downloads/my_debug_cloud.pcd -p frame_id:=base_link -p interval:=1.0
    declare_parameter("output_directory", "/home/ubuntu/Downloads/", "Directory to save output PCD files");    
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
    
    // Legacy...remove later Get parameters for plane segmentation
    max_plane_segmentation_iterations = this->get_parameter("max_plane_segmentation_iterations").as_int();
    plane_segmentation_distance_threshold = this->get_parameter("plane_segmentation_distance_threshold").as_double();
    
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

  // TODO
  bool segmentPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::ModelCoefficients::Ptr coefficients,
                    pcl::PointIndices::Ptr inliers) {
    if (!cloud || cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Input cloud is null or empty in segmentPlane()");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Starting plane segmentation. Input cloud size: %zu points", cloud->size());

    try {
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      
      // Set segmentation parameters
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(max_plane_segmentation_iterations);
      seg.setDistanceThreshold(plane_segmentation_distance_threshold);

      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) {
        RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "Plane segmentation completed. Found %zu inliers.", inliers->indices.size());
      RCLCPP_INFO(this->get_logger(), "Plane coefficients: a = %f, b = %f, c = %f, d = %f",
                  coefficients->values[0], coefficients->values[1], 
                  coefficients->values[2], coefficients->values[3]);

      // Remove the planar inliers, extract the rest
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*cloud);

      RCLCPP_INFO(this->get_logger(), "Remaining cloud after plane removal: %zu points.", cloud->size());

      return true;
    } catch (const pcl::PCLException& e) {
      RCLCPP_ERROR(this->get_logger(), "PCL exception caught during plane segmentation: %s", e.what());
      return false;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception caught during plane segmentation: %s", e.what());
      return false;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception caught during plane segmentation");
      return false;
    }
  }
  
  moveit_msgs::msg::CollisionObject createSupportSurfaceObject(
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
    
    // Create a shape_msgs::msg::Plane message with the coefficients
    shape_msgs::msg::Plane plane;
    for (size_t i = 0; i < 4; ++i) {
      plane.coef[i] = static_cast<double>(plane_coefficients->values[i]);
    }
    // Calculate an appropriate pose for the plane based on its normal vector
    geometry_msgs::msg::Pose plane_pose;
    Eigen::Vector3d normal(plane.coef[0], plane.coef[1], plane.coef[2]);
    normal.normalize();
    
    // Calculate rotation to align the plane normal with the z-axis
    Eigen::Quaterniond rotation;
    rotation.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
    
    // Set the pose
    plane_pose.position.x = 0;  // Adjust as needed
    plane_pose.position.y = 0;  // Adjust as needed
    plane_pose.position.z = -plane.coef[3] / normal.norm();  // Distance from origin
    plane_pose.orientation.x = rotation.x();
    plane_pose.orientation.y = rotation.y();
    plane_pose.orientation.z = rotation.z();
    plane_pose.orientation.w = rotation.w();
    
    // Set up the CollisionObject
    support_surface.header.frame_id = frame_id;
    support_surface.id = "support_surface";
    support_surface.planes.push_back(plane);
    support_surface.plane_poses.push_back(plane_pose);
    support_surface.operation = moveit_msgs::msg::CollisionObject::ADD;
    RCLCPP_INFO(this->get_logger(), "Support surface object created successfully");
    RCLCPP_INFO(this->get_logger(), "Support surface normal: [%.2f, %.2f, %.2f]", 
                 normal.x(), normal.y(), normal.z());
    RCLCPP_INFO(this->get_logger(), "Support surface position: [%.2f, %.2f, %.2f]",
                 plane_pose.position.x, plane_pose.position.y, plane_pose.position.z);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error creating support surface object: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error occurred while creating support surface object");
    }
    return support_surface;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    if (!cloud || cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Input cloud is null or empty in extractClusters()");
      return clusters;
    }
    RCLCPP_INFO(this->get_logger(), "Starting cluster extraction. Input cloud size: %zu points", cloud->size());
    try {
      // Remove NaN and Inf points
      auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);
      
      // Additional check for remaining invalid points
      auto cloud_valid = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      cloud_valid->points.reserve(cloud_filtered->points.size());
      for (const auto& point : cloud_filtered->points) {
        if (pcl::isFinite(point)) {
          cloud_valid->points.push_back(point);
        }
      }
      cloud_valid->width = cloud_valid->points.size();
      cloud_valid->height = 1;
      cloud_valid->is_dense = true;
      RCLCPP_INFO(this->get_logger(), "Removed invalid points. Filtered cloud size: %zu points", cloud_valid->size());
      if (cloud_valid->empty()) {
        RCLCPP_ERROR(this->get_logger(), "All points were invalid. Cannot proceed with clustering.");
        return clusters;
      }
      // Clustering
      auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
      tree->setInputCloud(cloud_valid);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
      ec.setClusterTolerance(cluster_tolerance);
      ec.setMinClusterSize(min_cluster_size);
      ec.setMaxClusterSize(max_cluster_size);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_valid);
      ec.extract(cluster_indices);
      RCLCPP_INFO(this->get_logger(), "Found %zu initial clusters", cluster_indices.size());
      // Process clusters
      for (const auto& indices : cluster_indices) {
        auto cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        cluster->points.reserve(indices.indices.size());
        for (const auto& index : indices.indices) {
          cluster->points.push_back((*cloud_valid)[index]);
        }
        cluster->width = static_cast<uint32_t>(cluster->points.size());
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
        RCLCPP_INFO(this->get_logger(), "Extracted cluster with %zu points", cluster->points.size());
      }
      RCLCPP_INFO(this->get_logger(), "Extraction complete. Returning %zu clusters", clusters.size());
    } catch (const pcl::PCLException& e) {
      RCLCPP_ERROR(this->get_logger(), "PCL exception caught during cluster extraction: %s", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception caught during cluster extraction: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception caught during cluster extraction");
    }
    return clusters;
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

  void handleService(
      const std::shared_ptr<mycobot_interfaces::srv::GetPlanningScene::Request> request,
      std::shared_ptr<mycobot_interfaces::srv::GetPlanningScene::Response> response) {
      
    // 1. Initialize response success flag to false
    response->success = false;

    // 2. Check if point cloud and RGB image data are available
    //    - If not, log an error and return early
    if (!latest_point_cloud || !latest_rgb_image) {
      RCLCPP_ERROR(this->get_logger(), "Point cloud or RGB image data not available");
      return;
    }

    // 3. Validate input parameters and prepare point cloud:
    //    - Check if target_shape and target_dimensions are not empty
    //    - Verify that target_shape is one of the valid shapes (cylinder, box)
    //    - Store the original point cloud frame for reference
    //    - Transform the point cloud to the target frame for consistent processing
    //    - Apply optional clipping (cropping) to the point cloud based on parameters
    //    - If any validation or transformation fails, log an error and return early
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

    // 4. Convert PointCloud2 to PCL point cloud
    //    - Check if conversion was successful and resulting cloud is not empty
    //    - If conversion fails, log an error and return early
    auto pcl_cloud = convertToPCL(transformed_cloud);
    if (!pcl_cloud) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert PointCloud2 to PCL format");
      return;
    }
    //  Used for debugging and visualization
    savePointCloudToPCD(pcl_cloud, "4_convertToPCL_" + debug_pcd_filename);

    // 5. Segment the support plane and objects from a given point cloud.
    auto [support_plane_cloud, objects_cloud] = segmentPlaneAndObjects(
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

    if (!support_plane_cloud || !objects_cloud) {
      RCLCPP_ERROR(this->get_logger(), "Plane and object segmentation failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Plane and object segmentation successful");
    RCLCPP_INFO(this->get_logger(), "Support plane cloud size: %zu", support_plane_cloud->size());
    RCLCPP_INFO(this->get_logger(), "Objects cloud size: %zu", objects_cloud->size());

    // For debugging
    savePointCloudToPCD(support_plane_cloud, "5_support_plane_" + debug_pcd_filename);
    savePointCloudToPCD(objects_cloud, "5_objects_cloud_" + debug_pcd_filename);

    // 6. Perform plane segmentation
    //    - Check if segmentation was successful
    //    - If segmentation fails, log an error and return early
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);

    bool plane_segmented = segmentPlane(pcl_cloud, plane_coefficients, plane_inliers);

    if (!plane_segmented) {
      RCLCPP_ERROR(this->get_logger(), "Plane segmentation failed");
    }

    RCLCPP_INFO(this->get_logger(), "Plane segmentation successful");

    // Check if the segmented plane is significant enough
    double plane_percentage = static_cast<double>(plane_inliers->indices.size()) / pcl_cloud->size();
    if (plane_percentage < plane_segmentation_threshold) { 
      RCLCPP_WARN(this->get_logger(), "Segmented plane is small (%.2f%% of points). It might not be the dominant plane.", plane_percentage * 100);
    }

    // 7. Create CollisionObject for support surface
    //    - Check if support surface object creation was successful
    //    - If creation fails, log a warning
    moveit_msgs::msg::CollisionObject support_surface = createSupportSurfaceObject(plane_coefficients, target_frame);
    
    if (support_surface.id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Support surface object creation failed or resulted in an invalid object");
    } else {
      RCLCPP_INFO(this->get_logger(), "Adding support surface to the planning scene");      
      // Add the support surface to the planning scene
      response->scene_world.collision_objects.push_back(support_surface);
    }

    // 8. Extract object clusters
    //    - Check if cluster extraction was successful and at least one cluster was found
    //    - If extraction fails or no clusters found, log an error and return early
    auto clusters = extractClusters(pcl_cloud);
    if (clusters.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No object clusters found in the point cloud");
    }
    RCLCPP_INFO(this->get_logger(), "Successfully extracted %zu object clusters", clusters.size());
    
    // 9. For each cluster:
    //    - Fit shapes and create CollisionObjects
    //    - Check if shape fitting was successful for at least one object
    //    - If no shapes could be fitted, log a warning
    bool any_shape_fitted = false;
    size_t shapes_fitted = 0;
    RCLCPP_INFO(this->get_logger(), "Processing %zu clusters for shape fitting", clusters.size());

    for (size_t i = 0; i < clusters.size(); ++i) {
      auto collision_object = fitShapeToCluster(clusters[i], target_frame, i);
      if (!collision_object.primitives.empty()) {
        response->scene_world.collision_objects.push_back(collision_object);
        any_shape_fitted = true;
        shapes_fitted++;
        RCLCPP_INFO(this->get_logger(), "Fitted shape for cluster %zu: %s", i, collision_object.id.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Failed to fit shape for cluster %zu", i);
      }
    }

    if (!any_shape_fitted) {
      RCLCPP_WARN(this->get_logger(), "No shapes could be fitted to any of the clusters");
    } else {
      RCLCPP_INFO(this->get_logger(), "Successfully fitted shapes to %zu out of %zu clusters", 
        shapes_fitted, clusters.size());
    }
    
    // 10. Identify target object
    //    - Check if a target object was successfully identified
    //    - If no target found, log a warning
    std::string target_object_id = identifyTargetObject(
      response->scene_world.collision_objects,
      request->target_shape,
      request->target_dimensions);

    if (!target_object_id.empty()) {
      response->target_object_id = target_object_id;
    } 
    
    // 11. Assemble PlanningSceneWorld
    //    - Check if assembly was successful
    //    - If assembly fails, log an error and return early
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
    
    // 12. Fill the rest of the response:
    //    - Set full_cloud, rgb_image, and target_object_id
    //    - Set success flag to true if all critical steps were successful
    response->full_cloud = *latest_point_cloud;  // Use the transformed cloud
    response->rgb_image = *latest_rgb_image;
    response->target_object_id = target_object_id;

  
    RCLCPP_INFO(this->get_logger(), "Success: %s", response->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Target object ID: %s", response->target_object_id.c_str());
  
    // Point cloud information
    RCLCPP_INFO(this->get_logger(), "Full cloud frame ID: %s", response->full_cloud.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Full cloud size: %d x %d", 
      response->full_cloud.width, response->full_cloud.height);
  
    // RGB image information
    RCLCPP_INFO(this->get_logger(), "RGB image frame ID: %s", response->rgb_image.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "RGB image size: %d x %d", 
      response->rgb_image.width, response->rgb_image.height);
      
    // Collision objects information
    RCLCPP_INFO(this->get_logger(), "Number of collision objects: %zu", 
      response->scene_world.collision_objects.size());
    for (const auto& obj : response->scene_world.collision_objects) {
      RCLCPP_INFO(this->get_logger(), "  Object ID: %s, Frame ID: %s", 
      obj.id.c_str(), obj.header.frame_id.c_str());
      RCLCPP_INFO(this->get_logger(), "    Number of primitives: %zu", obj.primitives.size());
      for (size_t i = 0; i < obj.primitives.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "    Primitive type: %d", obj.primitives[i].type);
        RCLCPP_INFO(this->get_logger(), "    Pose: [%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f, %.2f]",
          obj.primitive_poses[i].position.x, obj.primitive_poses[i].position.y, obj.primitive_poses[i].position.z,
          obj.primitive_poses[i].orientation.x, obj.primitive_poses[i].orientation.y, 
          obj.primitive_poses[i].orientation.z, obj.primitive_poses[i].orientation.w);
      }
    }  
    
    // Support surface information (if available)
    auto it = std::find_if(response->scene_world.collision_objects.begin(),
      response->scene_world.collision_objects.end(),
      [](const moveit_msgs::msg::CollisionObject& obj) { return obj.id == "support_surface"; });
    if (it != response->scene_world.collision_objects.end()) {
      RCLCPP_INFO(this->get_logger(), "Support surface detected:");
      RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", it->header.frame_id.c_str());
      if (!it->planes.empty()) {
        RCLCPP_INFO(this->get_logger(), "  Plane coefficients: [%.3f, %.3f, %.3f, %.3f]",
          it->planes[0].coef[0], it->planes[0].coef[1], it->planes[0].coef[2], it->planes[0].coef[3]);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No support surface detected in the scene.");
    }

    // Additional processing information
    RCLCPP_INFO(this->get_logger(), "Original point cloud frame: %s", latest_point_cloud->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Target frame used for processing: %s", target_frame.c_str());
    
    // Check if all critical steps were successful
    if (!response->scene_world.collision_objects.empty() &&
        !response->full_cloud.data.empty() &&
        !response->rgb_image.data.empty() &&
        !response->target_object_id.empty()) {
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Service response filled successfully");
    } else {
      response->success = false;
      RCLCPP_WARN(this->get_logger(), "Service response incomplete or invalid. Here is an explanation:");
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
    }
  
    RCLCPP_INFO(this->get_logger(), "Service response logging completed.");
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

