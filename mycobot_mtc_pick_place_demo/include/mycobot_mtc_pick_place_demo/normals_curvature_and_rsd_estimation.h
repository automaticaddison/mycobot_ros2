#ifndef NORMALS_CURVATURE_AND_RSD_ESTIMATION_H
#define NORMALS_CURVATURE_AND_RSD_ESTIMATION_H

/**
 * @file normals_curvature_and_rsd_estimation.h
 * @brief Estimate normal vectors, curvature values, and RSD values for each point in the point cloud.
 *
 * This file contains function declarations and type definitions for estimating normal vectors,
 * curvature values, and Radius-based Surface Descriptor (RSD) values for each point in a given point cloud.
 * It uses the PCL library for point cloud processing.
 *
 * Input:
 * - Point cloud (of type pcl::PointCloud<pcl::PointXYZRGB>::Ptr)
 * - Parameters for the estimation process (k_neighbors, max_plane_error, max_iterations, min_boundary_neighbors, rsd_radius)
 *
 * Output:
 * - Point Cloud with Normal Vectors, Curvature Values, and RSD Values (of type pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr)
 *   - Each point retains its original XYZ coordinates and RGB color information
 *   - Each point has an associated 3D vector representing its normal vector
 *   - Each point has a curvature value
 *   - Each point has RSD values (minimum and maximum surface radius)
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

// PCL library includes
#include <pcl/point_types.h>              // For basic point types
#include <pcl/point_cloud.h>              // For point cloud data structures
#include <pcl/search/kdtree.h>            // For K-d tree searches
#include <pcl/features/rsd.h>             // For RSD estimation
#include <pcl/common/pca.h>               // For Principal Component Analysis
#include <pcl/sample_consensus/sac_model_plane.h> // For plane model fitting
#include <pcl/sample_consensus/mlesac.h>          // For MLESAC algorithm
#include <pcl/impl/instantiate.hpp>               // For instantiating PCL classes
#include <pcl/search/impl/kdtree.hpp>             // For K-d tree implementation
#include <pcl/segmentation/region_growing.h>
#include <pcl/io/pcd_io.h>

// Eigen library include for linear algebra operations
#include <Eigen/Dense>

// Standard library includes
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <numeric>
#include <vector>

// Define a custom point type that includes RSD values
struct PointXYZRGBNormalRSD {
    PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;        // This adds the member rgb which can also be accessed using r, g, b
    PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using normal_x, normal_y, normal_z
    float curvature;    // Curvature value
    float r_min;        // Minimum radius for RSD
    float r_max;        // Maximum radius for RSD
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Ensure proper memory alignment for Eigen
} EIGEN_ALIGN16;                      // Enforce SSE padding for correct memory alignment

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBNormalRSD,
    (float, x, x)(float, y, y)(float, z, z)
    (float, rgb, rgb)
    (float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)
    (float, curvature, curvature)
    (float, r_min, r_min)(float, r_max, r_max)
)

// Instantiate PCL classes for the custom point type
PCL_INSTANTIATE_PRODUCT(KdTree, (pcl::PointXYZRGBNormalRSD))
PCL_INSTANTIATE_PRODUCT(Search, (pcl::PointXYZRGBNormalRSD))
PCL_INSTANTIATE_PRODUCT(RegionGrowing, ((pcl::PointXYZRGBNormalRSD))(pcl::Normal))

/**
 * @brief Log information to console.
 *
 * @param msg The message to log.
 */
void LOG_INFO(const std::string& msg);

/**
 * @brief Estimate normal vectors, curvature values, and RSD values for each point in the point cloud.
 *
 * @param input_cloud [pcl::PointCloud<pcl::PointXYZRGB>::Ptr] Input point cloud
 * @param k_neighbors [int] Number of neighbors to consider for each point
 * @param max_plane_error [double] Threshold for MLESAC plane fitting
 * @param max_iterations [int] Maximum iterations for MLESAC
 * @param min_boundary_neighbors [int] Minimum number of neighbors for boundary points
 * @param rsd_radius [double] Radius for RSD estimation
 * @return [pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr] Point Cloud with Normal Vectors, Curvature Values, and RSD Values
 */
pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr
estimateNormalsCurvatureAndRSD(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    int k_neighbors,
    double max_plane_error,
    int max_iterations,
    int min_boundary_neighbors,
    double rsd_radius);

#endif // NORMALS_CURVATURE_AND_RSD_ESTIMATION_H