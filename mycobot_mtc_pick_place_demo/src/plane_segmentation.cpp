/**
 * @file plane_segmentation.cpp
 * @brief Segment support plane and objects from a point cloud.
 *
 * This file contains a function to segment the support plane and objects from a given point cloud.
 * It uses surface normal estimation, Euclidean clustering, and RANSAC plane fitting to identify
 * the support surface and separate objects above it.
 *
 * Input:
 * - input_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr): The input point cloud
 * - enable_cropping (bool): Flag to enable or disable point cloud cropping
 * - crop_min_x, crop_max_x, crop_min_y, crop_max_y, crop_min_z, crop_max_z (double): Cropping boundaries
 * - max_iterations (int): Maximum iterations for RANSAC
 * - distance_threshold (double): Distance threshold for RANSAC
 * - z_tolerance (double): Tolerance for z-coordinate of the support plane
 * - angle_tolerance (double): Angle tolerance for surface normals
 * - min_cluster_size, max_cluster_size (int): Minimum and maximum size of clusters
 * - cluster_tolerance (double): Tolerance for Euclidean clustering
 * - normal_estimation_k (int): Number of neighbors for normal estimation
 * - plane_segmentation_threshold (double): Threshold for plane segmentation
 * - w_inliers, w_size, w_distance, w_orientation (double): Weights for plane model selection
 *
 * Output:
 * std::tuple containing:
 * - support_plane_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr): Point cloud of the segmented support plane
 * - objects_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr): Point cloud of objects above the support plane
 * - best_plane_model (pcl::ModelCoefficients::Ptr): Coefficients of the best-fit plane model
 *
 * The function processes the input point cloud, removes invalid points, estimates surface normals,
 * identifies potential support surfaces, performs clustering, and extracts the support plane and objects.
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

#include "mycobot_mtc_pick_place_demo/plane_segmentation.h"

// Function to segment the support plane and objects from a point cloud
std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients::Ptr>
segmentPlaneAndObjects(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    bool enable_cropping,
    double crop_min_x,
    double crop_max_x,
    double crop_min_y,
    double crop_max_y,
    double crop_min_z,
    double crop_max_z,
    int max_iterations,
    double distance_threshold,
    double z_tolerance,
    double angle_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    double cluster_tolerance,
    int normal_estimation_k,
    double plane_segmentation_threshold,
    double w_inliers,
    double w_size,
    double w_distance,
    double w_orientation) {

  LOG_INFO("Starting plane segmentation. Input cloud size: " << input_cloud->size() << " points");

  // Initialize return values
  auto support_plane_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  auto objects_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  auto best_plane_model = std::make_shared<pcl::ModelCoefficients>();

  // Remove NaN and Inf points
  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *cloud_filtered, indices);
  LOG_INFO("Removed NaN points. Filtered cloud size: " << cloud_filtered->size() << " points");

  // Additional check for remaining invalid points
  auto cleaned_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  cleaned_cloud->points.reserve(cloud_filtered->points.size());
  for (const auto& point : cloud_filtered->points) {
    if (pcl::isFinite(point)) {
      cleaned_cloud->points.push_back(point);
    }
  }
  cleaned_cloud->width = cleaned_cloud->points.size();
  cleaned_cloud->height = 1;
  cleaned_cloud->is_dense = true;

  LOG_INFO("Removed all invalid points. Final cleaned cloud size: " << cleaned_cloud->size() << " points");

  if (cleaned_cloud->empty()) {
    LOG_ERROR("All points were invalid. Cannot proceed with segmentation.");

    // Return empty clouds and a null plane model if all points were invalid
    return std::make_tuple(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
      pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients)
    );
  }

  // 1. Estimate surface normals
  LOG_INFO("Starting surface normal estimation");
  // A normal is a vector perpendicular to the surface at a given point
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setInputCloud(cleaned_cloud);
  ne.setSearchMethod(tree);
  // Use k-nearest neighbors for normal estimation
  ne.setKSearch(normal_estimation_k);
  ne.compute(*cloud_normals);
  LOG_INFO("Finished normal estimation. Computed " << cloud_normals->size() << " normals");

  // 2. Identify potential support surfaces
  // Use the computed surface normals to find approximately horizontal surfaces
  LOG_INFO("Identifying potential horizontal support surfaces (e.g., tables, shelves)");
  pcl::PointIndices::Ptr horizontal_indices(new pcl::PointIndices);
  for (size_t i = 0; i < cloud_normals->size(); ++i) {
    if (std::abs(cloud_normals->points[i].normal_z) > angle_tolerance) {
      horizontal_indices->indices.push_back(i);
    }
  }
  LOG_INFO("Found " << horizontal_indices->indices.size() << " points likely belonging to horizontal surfaces");

  if (horizontal_indices->indices.empty()) {
    LOG_ERROR("No horizontal surfaces found.");
    return std::make_tuple(support_plane_cloud, objects_cloud, best_plane_model);
  }
  // Extract the points with horizontal normals
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.setInputCloud(cleaned_cloud);
  extract.setIndices(horizontal_indices);
  extract.filter(*horizontal_cloud);
  LOG_INFO("Extracted horizontal cloud with " << horizontal_cloud->size() << " points");

  // 3. Perform Euclidean clustering on these points to get support surface candidate clusters
  LOG_INFO("Starting Euclidean clustering");
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  cluster_tree->setInputCloud(horizontal_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(cluster_tree);
  ec.setInputCloud(horizontal_cloud);
  ec.extract(cluster_indices);
  LOG_INFO("Finished clustering. Found " << cluster_indices.size() << " clusters");

  if (cluster_indices.empty()) {
    LOG_ERROR("No clusters found.");
    return std::make_tuple(support_plane_cloud, objects_cloud, best_plane_model);
  }

  // 4. Process each support surface candidate cluster
  LOG_INFO("Processing support surface candidate clusters");
  double best_score = -std::numeric_limits<double>::max();
  bool found_valid_plane = false;

  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    LOG_INFO("Processing cluster " << i+1 << " of " << cluster_indices.size());
    const auto& cluster = cluster_indices[i];

    // Extract the current cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> cluster_extract;
    cluster_extract.setInputCloud(horizontal_cloud);
    cluster_extract.setIndices(std::make_shared<const pcl::PointIndices>(cluster));
    cluster_extract.filter(*cluster_cloud);
    LOG_INFO("  Cluster size: " << cluster_cloud->size() << " points");

    // Use RANSAC to fit a plane model
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(cluster_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      LOG_INFO("  No plane model found for this cluster. Skipping.");
      continue;
    }
    LOG_INFO("  Fitted plane model. Inliers: " << inliers->indices.size());

    // Validate the plane model based on the robot's workspace limits
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f up_vector(0, 0, 1);
    double dot_product = plane_normal.dot(up_vector);

    Eigen::Vector4f plane_center;
    pcl::compute3DCentroid(*cluster_cloud, *inliers, plane_center);

    bool is_valid = true;

    if (enable_cropping) {
      is_valid = (plane_center[0] >= crop_min_x && plane_center[0] <= crop_max_x &&
                  plane_center[1] >= crop_min_y && plane_center[1] <= crop_max_y);
      LOG_INFO("  Cropping enabled. Plane center within bounds: " << (is_valid ? "Yes" : "No"));
    }
    is_valid = is_valid && (std::abs(plane_center[2]) < z_tolerance) && (dot_product > angle_tolerance);
    LOG_INFO("  Plane model validity: " << (is_valid ? "Valid" : "Invalid"));

    if (!is_valid) {
      LOG_INFO("  Plane model invalid. Details:");
      LOG_INFO("    Z-center: " << plane_center[2] << " (tolerance: " << z_tolerance << ")");
      LOG_INFO("    Normal-up dot product: " << dot_product << " (tolerance: " << angle_tolerance << ")");
      LOG_INFO("  Skipping.");
      continue;
    }

    // Calculate score for the plane model
    double inlier_count = static_cast<double>(inliers->indices.size());
    double inlier_score = inlier_count / cluster_cloud->size();
    double size_score = cluster_cloud->size() / static_cast<double>(cleaned_cloud->size());
    double distance_score = 1.0 - (std::abs(plane_center[2]) / z_tolerance);
    double orientation_score = dot_product;

    double total_score = w_inliers * inlier_score + w_size * size_score +
                         w_distance * distance_score + w_orientation * orientation_score;

    LOG_INFO("  Plane model scores - Inlier: " << inlier_score << ", Size: " << size_score
             << ", Distance: " << distance_score << ", Orientation: " << orientation_score
             << ", Total: " << total_score);

    if (total_score > best_score) {
      best_score = total_score;
      *best_plane_model = *coefficients;
      found_valid_plane = true;
      LOG_INFO("  New best plane model found. Score: " << best_score);
    }
  }

  LOG_INFO("Finished processing clusters. Best plane model score: " << best_score);

  if (!found_valid_plane) {
    LOG_ERROR("No valid plane model found.");
    LOG_ERROR("No valid plane model found.");
    // Add more detailed diagnostics
    LOG_ERROR("Please check z_tolerance (" << z_tolerance << ") and angle_tolerance ("
              << angle_tolerance << ") parameters");
    // Initialize a valid but empty response
    support_plane_cloud->width = 0;
    support_plane_cloud->height = 1;
    support_plane_cloud->is_dense = true;
    objects_cloud->width = 0;
    objects_cloud->height = 1;
    objects_cloud->is_dense = true;
    // Ensure the model has valid but empty coefficients
    best_plane_model->values.resize(4, 0.0);
    return std::make_tuple(support_plane_cloud, objects_cloud, best_plane_model);
  }

  // 5. Extract the support plane and objects above it
  LOG_INFO("Extracting support plane and objects");
  int support_plane_points = 0;
  int object_points = 0;

  for (const auto& point : cleaned_cloud->points) {
    double distance = best_plane_model->values[0] * point.x +
                      best_plane_model->values[1] * point.y +
                      best_plane_model->values[2] * point.z +
                      best_plane_model->values[3];

    if (std::abs(distance) < plane_segmentation_threshold) {
      support_plane_cloud->points.push_back(point);
      support_plane_points++;
    } else if (distance > 0) {
      if (!enable_cropping ||
          (point.x >= crop_min_x && point.x <= crop_max_x &&
           point.y >= crop_min_y && point.y <= crop_max_y &&
           point.z >= crop_min_z && point.z <= crop_max_z)) {
        objects_cloud->points.push_back(point);
        object_points++;
      }
    }
  }

  support_plane_cloud->width = support_plane_cloud->points.size();
  support_plane_cloud->height = 1;
  support_plane_cloud->is_dense = true;

  objects_cloud->width = objects_cloud->points.size();
  objects_cloud->height = 1;
  objects_cloud->is_dense = true;

  LOG_INFO("Segmentation complete. Support plane size: " << support_plane_points
           << " points, Objects cloud size: " << object_points << " points");
  LOG_INFO("Plane model coefficients: A=" << best_plane_model->values[0]
           << ", B=" << best_plane_model->values[1]
           << ", C=" << best_plane_model->values[2]
           << ", D=" << best_plane_model->values[3]);

  return std::make_tuple(support_plane_cloud, objects_cloud, best_plane_model);
}