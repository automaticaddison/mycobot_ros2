#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include "hello_mtc_with_perception/normals_curvature_and_rsd_estimation.h"  // For PointXYZRGBNormalRSD definition
#include <moveit_msgs/msg/collision_object.hpp>

/**
 * @brief Segment objects from point cloud clusters and create collision objects.
 * 
 * This function takes a vector of point cloud clusters, processes them to identify
 * geometric primitives (boxes or cylinders), and returns a vector of collision objects
 * representing these primitives.
 * 
 * @param cloud_clusters Vector of point cloud clusters to process
 * @param num_iterations Number of iterations for the inner loop
 * @param frame_id Frame ID for the collision objects
 * @param inlier_threshold Threshold for the number of inliers to consider a model valid
 * @return std::vector<moveit_msgs::msg::CollisionObject> Vector of collision objects
 */
std::vector<moveit_msgs::msg::CollisionObject> segmentObjects(
    const std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>& cloud_clusters,
    int num_iterations,
    const std::string& frame_id,
    double inlier_threshold);

// Helper function declarations will be added here as we develop them

#endif // OBJECT_SEGMENTATION_H
