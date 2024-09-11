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
 *     target_shape (string): Target object shape (e.g., "cylinder", "box", "cone", "sphere")
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
 * @date September 9, 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <mycobot_interfaces/srv/get_planning_scene.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
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
  
  // Parameters for point cloud preprocessing
  double voxel_leaf_size;
  int sor_mean_k;
  double sor_stddev_mult;
  
  // Parameters for plane segmentation
  int max_plane_segmentation_iterations;
  double plane_segmentation_distance_threshold;
  double plane_segmentation_threshold;

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
    
    // Declare parameters for point cloud preprocessing
    declare_parameter("voxel_leaf_size", 0.01, "Leaf size for VoxelGrid filter (in meters)"); // Decrease if you need more detail in the point cloud
    declare_parameter("sor_mean_k", 50, "Number of neighbors to analyze for each point in StatisticalOutlierRemoval"); // Increase if you need more detail in the point cloud
    declare_parameter("sor_stddev_mult", 1.0, "Standard deviation multiplier for StatisticalOutlierRemoval"); // Increase if you need more detail in the point cloud
    
    // Declare parameters for plane segmentation
    declare_parameter("max_plane_segmentation_iterations", 1000, "Maximum iterations for plane segmentation RANSAC");
    declare_parameter("plane_segmentation_distance_threshold", 0.01, "Distance threshold for plane segmentation (in meters)");  
    declare_parameter("plane_segmentation_threshold", 0.1, "Threshold for considering a plane significant (percentage of total points)");

    // Get parameter values
    point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
    rgb_image_topic = this->get_parameter("rgb_image_topic").as_string();
    target_frame = this->get_parameter("target_frame").as_string();
    
    voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
    sor_mean_k = this->get_parameter("sor_mean_k").as_int();
    sor_stddev_mult = this->get_parameter("sor_stddev_mult").as_double();
    
    max_plane_segmentation_iterations = this->get_parameter("max_plane_segmentation_iterations").as_int();
    plane_segmentation_distance_threshold = this->get_parameter("plane_segmentation_distance_threshold").as_double();
    plane_segmentation_threshold = this->get_parameter("plane_segmentation_threshold").as_double();
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
      return cloud_msg;
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

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> preprocessPointCloud(
      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud) {
    
    // Error handling for input cloud
    if (!cloud || cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Input point cloud is null or empty");
      return nullptr;
    }

    RCLCPP_INFO(this->get_logger(), "Starting point cloud preprocessing. Input cloud size: %zu points", cloud->size());

    auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    try {
      // Get the current parameter values
      double voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
      int sor_mean_k = this->get_parameter("sor_mean_k").as_int();
      double sor_stddev_mult = this->get_parameter("sor_stddev_mult").as_double();

      // 1. Apply VoxelGrid filter for downsampling
      pcl::VoxelGrid<pcl::PointXYZRGB> vox;
      vox.setInputCloud(cloud);
      vox.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
      vox.filter(*cloud_filtered);

      RCLCPP_INFO(this->get_logger(), "VoxelGrid filter applied. Cloud size after downsampling: %zu points", cloud_filtered->size());

      if (cloud_filtered->empty()) {
        RCLCPP_WARN(this->get_logger(), "VoxelGrid filter resulted in an empty cloud. Adjusting leaf size may be necessary.");
        return nullptr;
      }

      // 2. Apply StatisticalOutlierRemoval to clean the point cloud and remove noise
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(cloud_filtered);
      sor.setMeanK(sor_mean_k);
      sor.setStddevMulThresh(sor_stddev_mult);
      
      auto cloud_filtered_sor = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      sor.filter(*cloud_filtered_sor);

      RCLCPP_INFO(this->get_logger(), "StatisticalOutlierRemoval applied. Cloud size after statistical outlier removal: %zu points", cloud_filtered_sor->size());

      if (cloud_filtered_sor->empty()) {
        RCLCPP_WARN(this->get_logger(), "StatisticalOutlierRemoval resulted in an empty cloud. Adjusting parameters may be necessary.");
        return nullptr;
      }

      // Calculate point reduction percentage
      double reduction_percentage = ((cloud->size() - cloud_filtered_sor->size()) / static_cast<double>(cloud->size())) * 100.0;
      RCLCPP_INFO(this->get_logger(), "Total point reduction: %.2f%%", reduction_percentage);

      return cloud_filtered_sor;

    } catch (const pcl::PCLException& e) {
      RCLCPP_ERROR(this->get_logger(), "PCL exception caught during preprocessing: %s", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception caught during preprocessing: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception caught during preprocessing");
    }

    return nullptr;
  }

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
      [[maybe_unused]] pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    // TODO: Implement cluster extraction using Euclidean Cluster Extraction
    // 1. Set up clustering parameters
    // 2. Perform clustering
    // 3. Apply RANSAC iteratively to fit multiple geometric primitives
    // 4. For each RANSAC iteration:
    //    - Attempt to fit different shape models (cylinder, sphere, cone, box)
    //    - Select the best-fitting model based on inliers and fit quality
    //    - Extract the inliers of the best-fitting model as a distinct object
    //    - Remove the inliers from the point cloud and repeat the process
    // 5. Adjust RANSAC parameters based on expected object size and complexity
    // 6. Continue until a specified percentage of points are assigned to objects or max objects reached
    //    (e.g., 90% of points assigned or maximum of 10 objects detected)
    // 7. Return vector of clusters
    return {};  // Placeholder return
  }

  moveit_msgs::msg::CollisionObject fitShapeToCluster(
      [[maybe_unused]] pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster,
      [[maybe_unused]] const std::string& frame_id,
      [[maybe_unused]] int index) {
    // TODO: Implement shape fitting
    // 1. Attempt to fit multiple primitive shapes (box, sphere, cylinder, cone) using RANSAC
    // 2. Compare fitted shapes based on:
    //    - Number of inliers
    //    - Fitness score (e.g., sum of squared distances from points to the model)
    //    - Similarity to target object dimensions
    // 3. Select the shape with the best fit
    // 4. Create and return a CollisionObject for the best-fitting shape:
    //    - Set header.frame_id to the desired frame
    //    - Set id field to a unique string: "<shape_type>_<number>", e.g., "cylinder_1"
    //    - Create a shape_msgs::msg::SolidPrimitive with appropriate type and dimensions
    //    - Set the pose based on the cluster's centroid and orientation
    //    - Set operation field to ADD (0)
    return moveit_msgs::msg::CollisionObject();  // Placeholder return
  }

  std::string identifyTargetObject(
      [[maybe_unused]] const std::vector<moveit_msgs::msg::CollisionObject>& objects,
      [[maybe_unused]] const std::string& target_shape,
      [[maybe_unused]] const std::vector<double>& target_dimensions) {
    // TODO: Implement target object identification
    // 1. For each fitted shape, compare its dimensions to the input target dimensions
    // 2. Calculate a similarity score based on the shape type and dimension differences
    // 3. Select the object with the highest similarity score as the target object
    // 4. Return the ID of the best matching object
    return "";  // Placeholder return
  }

  moveit_msgs::msg::PlanningSceneWorld assemblePlanningSceneWorld(
      [[maybe_unused]] const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects) {
    // TODO: Implement PlanningSceneWorld assembly
    // 1. Create a moveit_msgs::msg::PlanningSceneWorld message
    // 2. Add all CollisionObjects to the collision_objects field of PlanningSceneWorld
    // 3. Ensure all CollisionObjects are in the desired frame (transform if necessary)
    return moveit_msgs::msg::PlanningSceneWorld();  // Placeholder return
  }
  
  void handleService(
      const std::shared_ptr<mycobot_interfaces::srv::GetPlanningScene::Request> request,
      std::shared_ptr<mycobot_interfaces::srv::GetPlanningScene::Response> response) {
    // TODO: Implement the main logic for processing the service request with error handling
    // 1. Initialize response success flag to false
    response->success = false;

    // 2. Check if point cloud and RGB image data are available
    //    - If not, log an error and return early
    if (!latest_point_cloud || !latest_rgb_image) {
      RCLCPP_ERROR(this->get_logger(), "Point cloud or RGB image data not available");
      return;
    }

    // 3. Validate input parameters:
    //    - Check if target_shape is not empty
    //    - Check if target_dimensions is not empty
    //    - Verify that target_shape is one of the valid shapes (cylinder, box, cone, sphere)
    //    - If any validation fails, log an error and return early
    if (request->target_shape.empty() || request->target_dimensions.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid input parameters: target_shape or target_dimensions is empty");
      return;
    }

    const std::vector<std::string> valid_shapes = {"cylinder", "box", "cone", "sphere"};
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

    // 5. Preprocess the point cloud
    //    - Check if preprocessing was successful and resulting cloud is not empty
    //    - If preprocessing fails, log an error and return early
    auto preprocessed_cloud = preprocessPointCloud(pcl_cloud);
    if (!preprocessed_cloud || preprocessed_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Point cloud preprocessing failed or resulted in an empty cloud");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Point cloud preprocessing successful. Preprocessed cloud size: %zu points", preprocessed_cloud->size());

    // 6. Perform plane segmentation
    //    - Check if segmentation was successful
    //    - If segmentation fails, log an error and return early
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);

    bool plane_segmented = segmentPlane(preprocessed_cloud, plane_coefficients, plane_inliers);

    if (!plane_segmented) {
      RCLCPP_ERROR(this->get_logger(), "Plane segmentation failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Plane segmentation successful");

    // Check if the segmented plane is significant enough
    double plane_percentage = static_cast<double>(plane_inliers->indices.size()) / preprocessed_cloud->size();
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
      RCLCPP_INFO(this->get_logger(), "Support surface object created successfully");      
      // Add the support surface to the planning scene
      response->scene_world.collision_objects.push_back(support_surface);
    }

    // 8. Extract object clusters
    //    - Check if cluster extraction was successful and at least one cluster was found
    //    - If extraction fails or no clusters found, log an error and return early

    // 9. For each cluster:
    //    - Fit shapes and create CollisionObjects
    //    - Check if shape fitting was successful for at least one object
    //    - If no shapes could be fitted, log a warning

    // 10. Identify target object
    //    - Check if a target object was successfully identified
    //    - If no target found, log a warning

    // 11. Assemble PlanningSceneWorld
    //    - Check if assembly was successful
    //    - If assembly fails, log an error and return early

    // 12. Fill the response:
    //    - Set scene_world, full_cloud, rgb_image, and target_object_id
    //    - Set success flag to true if all critical steps were successful

    // For now, we'll just create a dummy PlanningSceneWorld with a single object
    moveit_msgs::msg::PlanningSceneWorld scene_world;
    moveit_msgs::msg::CollisionObject object;
    object.id = "dummy_object";
    object.header.frame_id = target_frame;

    shape_msgs::msg::SolidPrimitive primitive;
    if (request->target_shape == "cylinder") {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[primitive.CYLINDER_HEIGHT] = request->target_dimensions[0];
      primitive.dimensions[primitive.CYLINDER_RADIUS] = request->target_dimensions[1];
    } else if (request->target_shape == "box") {
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = request->target_dimensions[0];
      primitive.dimensions[primitive.BOX_Y] = request->target_dimensions[1];
      primitive.dimensions[primitive.BOX_Z] = request->target_dimensions[2];
    } else if (request->target_shape == "sphere") {
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions.resize(1);
      primitive.dimensions[primitive.SPHERE_RADIUS] = request->target_dimensions[0];
    } else if (request->target_shape == "cone") {
      primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
      primitive.dimensions.resize(2);
      primitive.dimensions[primitive.CONE_HEIGHT] = request->target_dimensions[0];
      primitive.dimensions[primitive.CONE_RADIUS] = request->target_dimensions[1];
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported shape: %s", request->target_shape.c_str());
      return;
    }

    object.primitives.push_back(primitive);

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    object.primitive_poses.push_back(pose);

    scene_world.collision_objects.push_back(object);

    response->scene_world = scene_world;
    response->full_cloud = *latest_point_cloud;
    response->rgb_image = *latest_rgb_image;
    response->target_object_id = "dummy_object";
    response->success = true;

    // 13. Log appropriate messages for successful operation or any warnings
    RCLCPP_INFO(this->get_logger(), "Service call completed successfully");

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

