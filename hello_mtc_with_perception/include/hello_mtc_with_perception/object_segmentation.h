#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

#include <cmath>                                  // For mathematical operations
#include <tuple>                                  // For std::tuple in function returns
#include <unordered_map>                          // For projection_map
#include <vector>                                 // For std::vector
#include <limits>                                 // For std::numeric_limits
#include <algorithm>                              // For std::min, std::max, std::set_intersection
#include <sstream>                                // For std::ostringstream in logging
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

#include "hello_mtc_with_perception/normals_curvature_and_rsd_estimation.h" // For LOG_INFO implementation

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
 * @param line_coefficients Coefficients of the fitted line.
 * @param projection_map Mapping between 2D projected points and original 3D points.
 * @param line_min_cluster_size Minimum number of points for a valid cluster.
 * @param line_max_clusters Maximum number of allowed clusters.
 * @param line_curvature_threshold Threshold for point curvature.
 * @param line_normal_angle_threshold Threshold for angle between point normal and line normal.
 * @param line_cluster_tolerance The maximum distance between two points to be considered in the same cluster.
 * @return pcl::PointIndices::Ptr Filtered set of inlier indices.
 */
pcl::PointIndices::Ptr filterLineInliers(
    const pcl::PointIndices::Ptr& line_inliers,
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& original_cloud,
    const pcl::ModelCoefficients::Ptr& line_coefficients,
    const std::unordered_map<size_t, size_t>& projection_map,
    int line_min_cluster_size,
    int line_max_clusters,
    double line_curvature_threshold,
    double line_normal_angle_threshold,
    double line_cluster_tolerance);

/**
 * @brief Segments objects from point cloud clusters and creates collision objects.
 *
 * This function processes a vector of point cloud clusters to identify geometric primitives
 * (cylinder and box) and returns a vector of collision objects
 * representing these primitives.
 *
 * @param cloud_clusters Vector of point cloud clusters to process.
 * @param num_iterations Number of iterations for the inner loop of processing each cluster.
 * @param frame_id Frame ID for the collision objects.
 * @param inlier_threshold Threshold for the number of inliers to consider a model valid.
 * @param hough_angle_bins Number of angle bins for the line Hough space.
 * @param hough_rho_bins Number of distance bins for the line Hough space.
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
 * @param line_min_cluster_size Minimum number of points for a valid cluster.
 * @param line_max_clusters Maximum number of allowed clusters for lines.
 * @param line_curvature_threshold Threshold for point curvature in line fitting.
 * @param line_normal_angle_threshold Threshold for angle between point normal and line normal.
 * @param line_cluster_tolerance The maximum distance between two points to be considered in the same cluster for lines.
 * @return std::vector<moveit_msgs::msg::CollisionObject> Vector of collision objects.
 */
std::vector<moveit_msgs::msg::CollisionObject> segmentObjects(
    const std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>& cloud_clusters,
    int num_iterations,
    const std::string& frame_id,
    int inlier_threshold,
    int hough_angle_bins,
    int hough_rho_bins,
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
    double line_normal_angle_threshold,
    double line_cluster_tolerance);

#endif // OBJECT_SEGMENTATION_H