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
 * @return std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> 
 *         A tuple containing the inlier indices and line coefficients.
 */
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitLineRANSAC(
  const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
  double ransac_distance_threshold,
  int ransac_max_iterations);
  
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
 * @return std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> 
 *         A tuple containing the inlier indices and circle coefficients.
 */
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitCircleRANSAC(
  const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
  double ransac_distance_threshold,
  int ransac_max_iterations,
  double max_allowable_radius);

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
 * @brief Segments objects from point cloud clusters and creates collision objects.
 * 
 * This function processes a vector of point cloud clusters to identify geometric primitives 
 * (currently only cylinders are implemented) and returns a vector of collision objects 
 * representing these primitives. It uses RANSAC for object detection and fitting.
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
    int ransac_max_iterations);

#endif // OBJECT_SEGMENTATION_H
