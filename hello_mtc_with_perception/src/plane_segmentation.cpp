/**
 * @file plane_segmentation.cpp
 * @brief Segment support plane and objects from a point cloud.
 *
 * This file contains a function to segment the support plane and objects from a given point cloud.
 * It uses surface normal estimation, Euclidean clustering, and RANSAC plane fitting to identify
 * the support surface and separate objects above it.
 *
 * @author Addison Sears-Collins
 * @date September 17, 2024
 */

#include "hello_mtc_with_perception/plane_segmentation.h"

// Function to segment the support plane and objects from a point cloud
std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> 
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
		
  // 1. Estimate surface normals
  // A normal is a vector perpendicular to the surface at a given point
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setInputCloud(input_cloud);
  ne.setSearchMethod(tree);
  // Use k-nearest neighbors for normal estimation
  ne.setKSearch(normal_estimation_k);
  ne.compute(*cloud_normals);

  // 2. Identify potential support surfaces
  // Use the computed surface normals to find approximately horizontal surfaces
  pcl::PointIndices::Ptr horizontal_indices(new pcl::PointIndices);
  for (size_t i = 0; i < cloud_normals->size(); ++i) {
    // Group points whose normals are approximately parallel to the world Z-axis (vertical)
    if (std::abs(cloud_normals->points[i].normal_z) > angle_tolerance) {
      horizontal_indices->indices.push_back(i);
    }
  }

  // Extract the points with horizontal normals
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.setInputCloud(input_cloud);
  extract.setIndices(horizontal_indices);
  extract.filter(*horizontal_cloud);

  // 3. Perform Euclidean clustering on these points to get support surface candidate clusters
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

  // 4. Process each support surface candidate cluster
  pcl::ModelCoefficients::Ptr best_plane_model(new pcl::ModelCoefficients);
  double best_score = -std::numeric_limits<double>::max();

  for (const auto& cluster : cluster_indices) {
    // Extract the current cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> cluster_extract;
    cluster_extract.setInputCloud(horizontal_cloud);
    cluster_extract.setIndices(std::make_shared<const pcl::PointIndices>(cluster));
    cluster_extract.filter(*cluster_cloud);

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

    if (inliers->indices.empty()) continue;

    // Validate the plane model based on the robot's workspace limits
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f up_vector(0, 0, 1);
    double dot_product = plane_normal.dot(up_vector);

    Eigen::Vector4f plane_center;
    pcl::compute3DCentroid(*cluster_cloud, *inliers, plane_center);

    bool is_valid = true;
	
    // Check if the plane is within the cropped area if cropping is enabled
    if (enable_cropping) {
      is_valid = (plane_center[0] >= crop_min_x && plane_center[0] <= crop_max_x &&
                  plane_center[1] >= crop_min_y && plane_center[1] <= crop_max_y);
    }
    // Check if the plane is close to z=0 and approximately horizontal
    is_valid = is_valid && (std::abs(plane_center[2]) < z_tolerance) && (dot_product > angle_tolerance);

    if (!is_valid) continue;

    // Calculate score for the plane model
    double inlier_count = static_cast<double>(inliers->indices.size());
    double inlier_score = inlier_count / cluster_cloud->size();
    double size_score = cluster_cloud->size() / static_cast<double>(input_cloud->size());
    double distance_score = 1.0 - (std::abs(plane_center[2]) / z_tolerance);
    double orientation_score = dot_product;

    // Combine factors using a weighted scoring system
    double total_score = w_inliers * inlier_score + w_size * size_score + 
                         w_distance * distance_score + w_orientation * orientation_score;

    // Select the plane model with the highest total_score as the best fitting plane
    if (total_score > best_score) {
      best_score = total_score;
      *best_plane_model = *coefficients;
    }
  }

  // 5. Extract the support plane and objects above it
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr support_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto& point : input_cloud->points) {
	  
    // Use the plane equation to check if the point is above the best_plane_model
    double distance = best_plane_model->values[0] * point.x +
                      best_plane_model->values[1] * point.y +
                      best_plane_model->values[2] * point.z +
                      best_plane_model->values[3];

    if (std::abs(distance) < plane_segmentation_threshold) {
		
      // Point belongs to the support plane
      support_plane_cloud->points.push_back(point);
	  
    } else if (distance > 0) {
		
      // Point is above the support plane
      if (!enable_cropping || 
          (point.x >= crop_min_x && point.x <= crop_max_x &&
           point.y >= crop_min_y && point.y <= crop_max_y &&
           point.z >= crop_min_z && point.z <= crop_max_z)) {
        objects_cloud->points.push_back(point);
      }
    }
  }

  // Set the width, height, and is_dense properties for the output point clouds
  support_plane_cloud->width = support_plane_cloud->points.size();
  support_plane_cloud->height = 1;
  support_plane_cloud->is_dense = true;

  objects_cloud->width = objects_cloud->points.size();
  objects_cloud->height = 1;
  objects_cloud->is_dense = true;

  // Return both support_plane_cloud and objects_cloud
  return std::make_pair(support_plane_cloud, objects_cloud);
}

