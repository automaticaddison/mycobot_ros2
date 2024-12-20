#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

/**
 * @file object_segmentation.h
 * @brief Header file for object segmentation algorithms and utilities.
 *
 * This file contains declarations for functions and structures used in segmenting
 * 3D point cloud data into geometric primitives (cylinders and boxes). It includes
 * methods for RANSAC-based model fitting, Hough transform techniques, and various
 * filtering operations.
 *
 * Key Features:
 *     - RANSAC-based line and circle fitting
 *     - Point cloud projection and filtering
 *     - Hough transform for model voting and clustering
 *     - Cylinder and box fitting to 3D point clouds
 *     - Creation of collision objects for MoveIt
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

#include <cmath>                                  // For mathematical operations
#include <tuple>                                  // For std::tuple in function returns
#include <unordered_map>                          // For projection_map
#include <vector>                                 // For std::vector
#include <limits>                                 // For std::numeric_limits
#include <algorithm>                              // For std::min, std::max, std::set_intersection
#include <sstream>                                // For std::ostringstream in logging
#include <set>                                    // For removing inliers of valid models from a point cloud cluster
#include <random>                                 // For RANSAC-based model fitting

#include <Eigen/Dense>                            // For Eigen::MatrixXi
#include <unsupported/Eigen/CXX11/Tensor>         // For Eigen::Tensor

#include <pcl/point_types.h>                      // For pcl::PointXY, PointXYZRGBNormalRSD
#include <pcl/point_cloud.h>                      // For pcl::PointCloud
#include <pcl/common/io.h>                        // For pcl::copyPointCloud
#include <pcl/common/projection_matrix.h>         // For point cloud projection
#include <pcl/search/kdtree.h>                    // For pcl::search::KdTree
#include <pcl/segmentation/extract_clusters.h>    // For pcl::EuclideanClusterExtraction

#include <moveit_msgs/msg/collision_object.hpp>   // For moveit_msgs::msg::CollisionObject
#include <geometry_msgs/msg/pose.hpp>            // For geometry_msgs::msg::Pose
#include <shape_msgs/msg/solid_primitive.hpp>    // For shape_msgs::msg::SolidPrimitive

#include <tf2/LinearMath/Quaternion.h>  // For quaternion operations
#include <tf2/LinearMath/Matrix3x3.h>   // For conversion between quaternions and Euler angles

#include "mycobot_mtc_pick_place_demo/normals_curvature_and_rsd_estimation.h" // For LOG_INFO implementation

/**
 * @brief Fits a 2D line to two points.
 *
 * @param p1 First point.
 * @param p2 Second point.
 * @return Eigen::Vector3f Line coefficients (a, b, c) where ax + by + c = 0.
 */
Eigen::Vector3f fitLine(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2);

/**
 * @brief Calculates the distance from a point to a line.
 *
 * @param point The point to calculate the distance from.
 * @param line The line coefficients (a, b, c) where ax + by + c = 0.
 * @return float The distance from the point to the line.
 */
float distanceToLine(const Eigen::Vector2f& point, const Eigen::Vector3f& line);

/**
 * @brief Fits a 2D line to a point cloud using RANSAC.
 *
 * @param cloud Input point cloud.
 * @param ransac_distance_threshold Distance threshold for inlier determination.
 * @param ransac_max_iterations Maximum number of RANSAC iterations.
 * @param projection_map Mapping between 2D projected points and original 3D points.
 * @return std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>
 *         A tuple containing the inlier indices and line coefficients.
 */
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitLineRANSAC(
  const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
  double ransac_distance_threshold,
  int ransac_max_iterations,
  const std::unordered_map<size_t, size_t>& projection_map);

/**
 * @brief Fits a 2D circle to three points.
 *
 * @param p1 First point.
 * @param p2 Second point.
 * @param p3 Third point.
 * @return Eigen::Vector3f Circle parameters (center_x, center_y, radius).
 */
Eigen::Vector3f fitCircle(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& p3);

/**
 * @brief Calculates the distance from a point to a circle.
 *
 * @param point The point to calculate the distance from.
 * @param circle The circle parameters (center_x, center_y, radius).
 * @return float The distance from the point to the circle.
 */
float distanceToCircle(const Eigen::Vector2f& point, const Eigen::Vector3f& circle);

/**
 * @brief Fits a 2D circle to a point cloud using RANSAC.
 *
 * @param cloud Input point cloud.
 * @param ransac_distance_threshold Distance threshold for inlier determination.
 * @param ransac_max_iterations Maximum number of RANSAC iterations.
 * @param max_allowable_radius Maximum allowable radius for the fitted circle.
 * @param projection_map Mapping between 2D projected points and original 3D points.
 * @return std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>
 *         A tuple containing the inlier indices and circle coefficients.
 */
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitCircleRANSAC(
  const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
  double ransac_distance_threshold,
  int ransac_max_iterations,
  double max_allowable_radius,
  const std::unordered_map<size_t, size_t>& projection_map);

/**
 * @brief Logs the results of model fitting.
 *
 * @param modelType The type of model ("Line" or "Circle").
 * @param coefficients The coefficients of the fitted model.
 * @param inliers The inlier points of the fitted model.
 */
void logModelResults(const std::string& modelType,
                     const pcl::ModelCoefficients::Ptr& coefficients,
                     const pcl::PointIndices::Ptr& inliers);

/**
 * @brief Filters circle inliers based on various criteria.
 *
 * This function applies several filters to refine the set of inlier points for a fitted circle:
 * 1. Uses Euclidean clustering to group inliers and limits the number of clusters.
 * 2. Checks for height consistency between clusters.
 * 3. Filters based on point curvature.
 * 4. Filters based on the Radius-based Surface Descriptor (RSD).
 * 5. Filters based on surface normal alignment with the circle.
 *
 * @param circle_inliers Initial set of inliers from RANSAC circle fitting.
 * @param original_cloud Original 3D point cloud.
 * @param circle_coefficients Coefficients of the fitted circle.
 * @param projection_map Mapping between 2D projected points and original 3D points.
 * @param circle_min_cluster_size Minimum size for a cluster of inliers.
 * @param circle_max_clusters Maximum number of allowed clusters.
 * @param circle_height_tolerance Tolerance for height difference between clusters.
 * @param circle_curvature_threshold Threshold for point curvature.
 * @param circle_radius_tolerance Tolerance for difference between point Radius-based Surface Descriptor (RSD) min value and circle radius.
 * @param circle_normal_angle_threshold Threshold for angle between point normal and circle radial vector.
 * @param cluster_tolerance The maximum distance between two points to be considered in the same cluster.
 * @return pcl::PointIndices::Ptr Filtered set of inlier indices.
 */
pcl::PointIndices::Ptr filterCircleInliers(
    const pcl::PointIndices::Ptr& circle_inliers,
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& original_cloud,
    const pcl::ModelCoefficients::Ptr& circle_coefficients,
    const std::unordered_map<size_t, size_t>& projection_map,
    int circle_min_cluster_size,
    int circle_max_clusters,
    double circle_height_tolerance,
    double circle_curvature_threshold,
    double circle_radius_tolerance,
    double circle_normal_angle_threshold,
    double circle_cluster_tolerance);

/**
 * @brief Filters line inliers based on various criteria.
 *
 * This function applies several filters to refine the set of inlier points for a fitted line:
 * 1. Uses Euclidean clustering to group inliers and limits the number of clusters.
 * 2. Filters based on point curvature.
 * 3. Filters based on surface normal alignment with the line.
 *
 * @param line_inliers Initial set of inliers from RANSAC line fitting.
 * @param original_cloud Original 3D point cloud.
 * @param projection_map Mapping between 2D projected points and original 3D points.
 * @param line_min_cluster_size Minimum number of points for a valid cluster.
 * @param line_max_clusters Maximum number of allowed clusters.
 * @param line_curvature_threshold Threshold for point curvature.
 * @param line_cluster_tolerance The maximum distance between two points to be considered in the same cluster.
 * @return pcl::PointIndices::Ptr Filtered set of inlier indices.
 */
pcl::PointIndices::Ptr filterLineInliers(
    const pcl::PointIndices::Ptr& line_inliers,
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& original_cloud,
    const std::unordered_map<size_t, size_t>& projection_map,
    int line_min_cluster_size,
    int line_max_clusters,
    double line_curvature_threshold,
    double line_cluster_tolerance);

/**
 * @brief Represents a validated model (circle or line) with its parameters and inlier indices.
 */
struct ValidModel {
  std::string type;  // "circle" or "line"
  std::vector<double> parameters;  // Store model parameters
  std::vector<int> inlier_indices;  // Store indices of inlier points
};

/**
 * @brief Represents a clustered Hough bin for lines or circles.
 */
struct HoughBin {
  std::vector<int> indices;
  int votes;
  int inlierCount;
  std::vector<double> parameters;
};

/**
 * @brief Represents a line model with its parameters and vote count.
 */
struct LineModel {
    double rho;
    double theta;
    int votes;
    int inlierCount;
};

/**
 * @brief Clusters line models based on their rho and theta values.
 *
 * @param lineModels Vector of line models to cluster.
 * @param rhoThreshold Threshold for considering two rho values similar.
 * @param thetaThreshold Threshold for considering two theta values similar.
 * @return std::vector<HoughBin> Vector of clustered line models.
 */
std::vector<HoughBin> clusterLineModels(
    const std::vector<LineModel>& lineModels,
    double rhoThreshold,
    double thetaThreshold);

/**
 * @brief Clusters the circle Hough space.
 *
 * @param houghSpaceCircle The 3D Hough space for circles.
 * @param houghCenterXStep Step size for the center x dimension in the Hough space.
 * @param houghCenterYStep Step size for the center y dimension in the Hough space.
 * @param houghRadiusStep Step size for the radius dimension in the Hough space.
 * @param minPt_x Minimum x coordinate of the original point cloud.
 * @param minPt_y Minimum y coordinate of the original point cloud.
 * @return std::vector<HoughBin> Vector of clustered circle models.
 */
std::vector<HoughBin> clusterCircleHoughSpace(
    const Eigen::Tensor<int, 3>& houghSpaceCircle,
    double houghCenterXStep,
    double houghCenterYStep,
    double houghRadiusStep,
    double minPt_x,
    double minPt_y);

/**
 * @brief Fits a cylinder to a 3D point cloud cluster.
 *
 * @param cluster The 3D point cloud cluster.
 * @param center_x The x-coordinate of the circle center in 2D.
 * @param center_y The y-coordinate of the circle center in 2D.
 * @param radius The radius of the circle in 2D.
 * @return std::tuple<shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose>
 *         A tuple containing the cylinder primitive and its pose.
 */
std::tuple<shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose>
fitCylinderToCluster(
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& cluster,
    double center_x,
    double center_y,
    double radius);

/**
 * @brief Fits a box to a 3D point cloud cluster based on a line model.
 *
 * @param cluster The 3D point cloud cluster.
 * @param rho The rho parameter of the line model (perpendicular distance from origin to line).
 * @param theta The theta parameter of the line model (angle between x-axis and normal vector to line).
 * @return std::tuple<shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose>
 *         A tuple containing the box primitive and its pose.
 */
std::tuple<shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose>
fitBoxToCluster(
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& cluster,
    double rho,
    double theta);

/**
 * @brief Segments objects from point cloud clusters and creates collision objects.
 *
 * This function processes a vector of point cloud clusters to identify geometric primitives
 * (cylinders and boxes) and returns a vector of collision objects representing these primitives.
 *
 * @param cloud_clusters Vector of point cloud clusters to process.
 * @param num_iterations Number of iterations for the inner loop of processing each cluster.
 * @param frame_id Frame ID for the collision objects.
 * @param inlier_threshold Threshold for the number of inliers to consider a model valid.
 * @param hough_radius_bins Number of radius bins for the circle Hough space.
 * @param hough_center_bins Number of center bins (in each dimension) for the circle Hough space.
 * @param ransac_distance_threshold Distance threshold for RANSAC.
 * @param ransac_max_iterations Maximum number of iterations for RANSAC.
 * @param circle_min_cluster_size Minimum size for a cluster of circle inliers.
 * @param circle_max_clusters Maximum number of allowed clusters for circles.
 * @param circle_height_tolerance Tolerance for height difference between circle clusters.
 * @param circle_curvature_threshold Threshold for point curvature in circle fitting.
 * @param circle_radius_tolerance Tolerance for difference between point RSD min value and circle radius.
 * @param circle_normal_angle_threshold Threshold for angle between point normal and circle radial vector.
 * @param circle_cluster_tolerance The maximum distance between two points to be considered in the same cluster for circles.
 * @param line_min_cluster_size Minimum number of points for a valid line cluster.
 * @param line_max_clusters Maximum number of allowed clusters for lines.
 * @param line_curvature_threshold Threshold for point curvature in line fitting.
 * @param line_cluster_tolerance The maximum distance between two points to be considered in the same cluster for lines.
 * @param line_rho_threshold Threshold for considering two rho values similar when clustering lines.
 * @param line_theta_threshold Threshold for considering two theta values similar when clustering lines.
 * @return std::vector<moveit_msgs::msg::CollisionObject> Vector of collision objects.
 */
std::vector<moveit_msgs::msg::CollisionObject> segmentObjects(
    const std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>& cloud_clusters,
    int num_iterations,
    const std::string& frame_id,
    int inlier_threshold,
    int hough_radius_bins,
    int hough_center_bins,
    double ransac_distance_threshold,
    int ransac_max_iterations,
    int circle_min_cluster_size,
    int circle_max_clusters,
    double circle_height_tolerance,
    double circle_curvature_threshold,
    double circle_radius_tolerance,
    double circle_normal_angle_threshold,
    double circle_cluster_tolerance,
    int line_min_cluster_size,
    int line_max_clusters,
    double line_curvature_threshold,
    double line_cluster_tolerance,
    double line_rho_threshold,
    double line_theta_threshold);

#endif // OBJECT_SEGMENTATION_H