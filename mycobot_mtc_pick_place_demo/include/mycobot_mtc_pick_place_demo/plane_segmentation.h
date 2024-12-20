#ifndef PLANE_SEGMENTATION_H
#define PLANE_SEGMENTATION_H

/**
 * @file plane_segmentation.h
 * @brief Segment support plane and objects from a point cloud, and return plane coefficients.
 *
 * This file contains a function to segment the support plane and objects from a given point cloud.
 * It uses surface normal estimation, Euclidean clustering, and RANSAC plane fitting to identify
 * the support surface, separate objects above it, and return the plane coefficients.
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

#include <iostream>
#include <memory>
#include <tuple>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>

// Logging macros
#define LOG_INFO(x) std::cout << "INFO: " << x << std::endl
#define LOG_ERROR(x) std::cerr << "ERROR: " << x << std::endl

/**
 * @brief Segment the support plane and objects from a point cloud, and return plane coefficients.
 *
 * This function takes an input point cloud and performs segmentation to identify
 * the support plane and objects above it. It uses surface normal estimation,
 * Euclidean clustering, and RANSAC plane fitting. The function returns the segmented
 * support plane, objects above it, and the coefficients of the best-fit plane.
 *
 * @param input_cloud The input point cloud to be segmented.
 * @param enable_cropping Flag to enable cropping of the point cloud.
 * @param crop_min_x Minimum x-coordinate for cropping.
 * @param crop_max_x Maximum x-coordinate for cropping.
 * @param crop_min_y Minimum y-coordinate for cropping.
 * @param crop_max_y Maximum y-coordinate for cropping.
 * @param crop_min_z Minimum z-coordinate for cropping.
 * @param crop_max_z Maximum z-coordinate for cropping.
 * @param max_iterations Maximum number of iterations for RANSAC.
 * @param distance_threshold Distance threshold for RANSAC.
 * @param z_tolerance Tolerance for z-coordinate of the support plane.
 * @param angle_tolerance Angle tolerance for surface normals.
 * @param min_cluster_size Minimum size of a cluster.
 * @param max_cluster_size Maximum size of a cluster.
 * @param cluster_tolerance Tolerance for Euclidean clustering.
 * @param normal_estimation_k Number of neighbors for normal estimation.
 * @param plane_segmentation_threshold Threshold for plane segmentation.
 * @param w_inliers Weight for inlier score in plane model selection.
 * @param w_size Weight for size score in plane model selection.
 * @param w_distance Weight for distance score in plane model selection.
 * @param w_orientation Weight for orientation score in plane model selection.
 *
 * @return A tuple containing:
 *         1. Pointer to the segmented support plane point cloud
 *         2. Pointer to the segmented objects point cloud
 *         3. Pointer to the ModelCoefficients of the best-fit plane
 */
std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients::Ptr>
segmentPlaneAndObjects(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    bool enable_cropping = false,
    double crop_min_x = -std::numeric_limits<double>::max(),
    double crop_max_x = std::numeric_limits<double>::max(),
    double crop_min_y = -std::numeric_limits<double>::max(),
    double crop_max_y = std::numeric_limits<double>::max(),
    double crop_min_z = -std::numeric_limits<double>::max(),
    double crop_max_z = std::numeric_limits<double>::max(),
    int max_iterations = 100,
    double distance_threshold = 0.01,
    double z_tolerance = 0.03,
    double angle_tolerance = cos(2.5 * M_PI / 180.0),
    int min_cluster_size = 100,
    int max_cluster_size = 25000,
    double cluster_tolerance = 0.02,
    int normal_estimation_k = 30,
    double plane_segmentation_threshold = 0.001,
    double w_inliers = 1.0,
    double w_size = 1.0,
    double w_distance = 1.0,
    double w_orientation = 1.0);

#endif // PLANE_SEGMENTATION_H