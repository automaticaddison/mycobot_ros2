/**
 * @file cluster_extraction.cpp
 * @brief Extract clusters from a point cloud using region growing algorithm.
 *
 * This file contains a function to extract clusters from a point cloud with XYZ, RGB,
 * normal vectors, curvature values, and Radius-based Surface Descriptor (RSD) values.
 * It uses the PCL library for point cloud processing and implements a region growing algorithm.
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

#include "mycobot_mtc_pick_place_demo/cluster_extraction.h"

std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>
extractClusters(
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& input_cloud,
    unsigned int min_cluster_size,
    unsigned int max_cluster_size,
    float smoothness_threshold,
    float curvature_threshold,
    unsigned int nearest_neighbors) {
  LOG_INFO("Starting cluster extraction. Input cloud size: " + std::to_string(input_cloud->size()) + " points");

  // Create a KdTree object for the search method of the extraction
  auto tree = std::make_shared<pcl::search::KdTree<PointXYZRGBNormalRSD>>();

  // Create indices for the full point cloud
  pcl::IndicesPtr indices(new std::vector<int>);
  indices->resize(input_cloud->size());
  for (std::size_t i = 0; i < input_cloud->size(); ++i) {
    (*indices)[i] = static_cast<int>(i);
  }

  // Extract normals from the input cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normals->resize(input_cloud->size());
  for (std::size_t i = 0; i < input_cloud->size(); ++i) {
    normals->points[i].normal_x = input_cloud->points[i].normal_x;
    normals->points[i].normal_y = input_cloud->points[i].normal_y;
    normals->points[i].normal_z = input_cloud->points[i].normal_z;
    normals->points[i].curvature = input_cloud->points[i].curvature;
  }

  // Create a RegionGrowing object
  pcl::RegionGrowing<PointXYZRGBNormalRSD, pcl::Normal> reg;
  reg.setMinClusterSize(min_cluster_size);
  reg.setMaxClusterSize(max_cluster_size);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(nearest_neighbors);
  reg.setInputCloud(input_cloud);
  reg.setInputNormals(normals);
  reg.setIndices(indices);
  reg.setSmoothnessThreshold(smoothness_threshold / 180.0f * static_cast<float>(M_PI));
  reg.setCurvatureThreshold(curvature_threshold);

  std::vector<pcl::PointIndices> cluster_indices;
  reg.extract(cluster_indices);

  std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr> clusters;
  for (const auto& indices : cluster_indices) {
    auto cluster = std::make_shared<pcl::PointCloud<PointXYZRGBNormalRSD>>();
    for (const auto& index : indices.indices) {
      cluster->points.push_back(input_cloud->points[index]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
  }

  // Log sample of output data structure
  const std::size_t total_clusters = clusters.size();
  const std::size_t sample_size = std::min(static_cast<std::size_t>(10), total_clusters);
  std::stringstream ss;
  ss << "Sample of output data structure (showing " << sample_size << " out of " << total_clusters << " clusters):\n";
  ss << std::left
     << std::setw(8) << "Cluster"
     << std::setw(10) << "Size"
     << std::setw(10) << "X"
     << std::setw(10) << "Y"
     << std::setw(10) << "Z"
     << std::setw(10) << "R"
     << std::setw(10) << "G"
     << std::setw(10) << "B"
     << std::setw(10) << "MinCurv"
     << std::setw(10) << "20Curv"
     << std::setw(10) << "AvgCurv"
     << std::setw(10) << "MedCurv"
     << std::setw(10) << "80Curv"
     << std::setw(10) << "MaxCurv"
     << std::setw(10) << "RMin"
     << std::setw(10) << "RMax"
     << "\n";

  for (std::size_t i = 0; i < sample_size; ++i) {
    const auto& cluster = clusters[i];
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    std::vector<float> curvatures;
    float min_curvature = std::numeric_limits<float>::max();
    float max_curvature = std::numeric_limits<float>::lowest();
    float sum_curvature = 0.0f;
    float avg_r = 0.0f, avg_g = 0.0f, avg_b = 0.0f;
    float avg_rsd_min = 0.0f, avg_rsd_max = 0.0f;

    for (const auto& point : cluster->points) {
      curvatures.push_back(point.curvature);
      min_curvature = std::min(min_curvature, point.curvature);
      max_curvature = std::max(max_curvature, point.curvature);
      sum_curvature += point.curvature;
      avg_r += point.r;
      avg_g += point.g;
      avg_b += point.b;
      avg_rsd_min += point.r_min;
      avg_rsd_max += point.r_max;
    }

    float cluster_size = static_cast<float>(cluster->size());
    float avg_curvature = sum_curvature / cluster_size;
    avg_r /= cluster_size;
    avg_g /= cluster_size;
    avg_b /= cluster_size;
    avg_rsd_min /= cluster_size;
    avg_rsd_max /= cluster_size;

    // Sort curvatures for percentile calculations
    std::sort(curvatures.begin(), curvatures.end());

    // Calculate percentile curvatures
    float percentile_20_curvature = curvatures[static_cast<size_t>(curvatures.size() * 0.2)];
    float median_curvature = curvatures[curvatures.size() / 2];
    float percentile_80_curvature = curvatures[static_cast<size_t>(curvatures.size() * 0.8)];

    ss << std::setw(8) << i
       << std::setw(10) << cluster->size()
       << std::fixed << std::setprecision(2)
       << std::setw(10) << centroid[0]
       << std::setw(10) << centroid[1]
       << std::setw(10) << centroid[2]
       << std::setw(10) << std::round(avg_r)
       << std::setw(10) << std::round(avg_g)
       << std::setw(10) << std::round(avg_b)
       << std::fixed << std::setprecision(3)
       << std::setw(10) << min_curvature
       << std::setw(10) << percentile_20_curvature
       << std::setw(10) << avg_curvature
       << std::setw(10) << median_curvature
       << std::setw(10) << percentile_80_curvature
       << std::setw(10) << max_curvature
       << std::setw(10) << avg_rsd_min
       << std::setw(10) << avg_rsd_max
       << "\n";
  }

  ss << "\nKey:\n"
     << "Cluster: Index of the cluster\n"
     << "Size: Number of points in the cluster\n"
     << "X, Y, Z: Coordinates of the cluster centroid\n"
     << "R, G, B: Average RGB color values of the cluster\n"
     << "MinCurv: Minimum curvature of the cluster\n"
     << "20Curv: 20th percentile curvature of the cluster\n"
     << "AvgCurv: Average curvature of the cluster\n"
     << "MedCurv: Median curvature of the cluster\n"
     << "80Curv: 80th percentile curvature of the cluster\n"
     << "MaxCurv: Maximum curvature of the cluster\n"
     << "RMin, RMax: Average minimum and maximum radii from Radius-Based Surface Descriptor (RSD)\n";

  LOG_INFO("Cluster extraction completed. Number of clusters found: " + std::to_string(clusters.size()));
  LOG_INFO("Sample output:\n" + ss.str());

  return clusters;
}