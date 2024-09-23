#include "hello_mtc_with_perception/object_segmentation.h"

std::vector<moveit_msgs::msg::CollisionObject> segmentObjects(
    const std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>& cloud_clusters,
    int num_iterations,
    const std::string& frame_id,
    double inlier_threshold) {
  
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  [[maybe_unused]] int box_count = 0;
  int cylinder_count = 0;

  for ([[maybe_unused]] const auto& cluster : cloud_clusters) {
    // Silence unused variable warnings
    [[maybe_unused]] int num_iterations_copy = num_iterations;
    [[maybe_unused]] double inlier_threshold_copy = inlier_threshold;

    // TODO: Project the Point Cloud Cluster onto the Surface Plane
    // For each point (x,y,z,RGB,normal, curvature, RSDmin and RSDmax) in the 3D cluster:
    // - Project the point onto the surface plane (assumed to be z=0). This will create a point (x, y).
    // - Maintain a mapping between each 2D projected point and its original 3D point

    // TODO: Initialize two separate parameter spaces:
    // - Create an empty 2D Hough parameter space for line models
    // - Create an empty 2D Hough parameter space for circle models

    // TODO: Inner Loop (repeated I=25 times)
    for (int i = 0; i < num_iterations; ++i) {
      // TODO: RANSAC Model Fitting (pcl::RandomSampleConsensus)
      // Line Fitting
      // - Use RANSAC to fit a 2D line to the projected points. This is done to identify potential box-like objects (lines).
      // Circle Fitting
      // - Use RANSAC to fit a 2D circle to the projected points. This is done to identify cylindrical-like objects (cylinders)

      // TODO: Temporarily Remove Inliers
      // - Temporarily remove the inliers (for both the circle and line models) from this working point cloud cluster. This removal is just temporary.
      // For the next iteration I, we will begin again with the full point cloud cluster.

      // TODO: Filter Inliers
      // For the fitted model from the RANSAC Model Fitting step, apply a series of filters to refine the corresponding set of inlier points.
      // Circle Filtering
      // - Maximum of Two Clusters
      // - Height Consistency
      // - Curvature Filtering
      // - Radius-based Surface Descriptor Filtering
      // - Surface Normal Filtering
      // Line Filtering
      // - Maximum of 1 Cluster
      // - Curvature Filtering
      // - Surface Normal Filtering

      // TODO: Model Validation
      // For the circle and line models, check how many inlier points remain.
      // - If the number of remaining inliers for the model exceeds the threshold (100 points):
      //   - The model is considered valid.
      //   - Go to the next step to add the model to the circle or line Hough parameter space (depending on the model)
      // - If the number of remaining inliers is below the threshold:
      //   - The model is rejected.
      //   - Check if there are any remaining points in the cluster. If so, go back to the RANSAC Model Fitting step.

      // TODO: Add Model to the Hough Space (pcl::Hough3DGrouping)
      // If a circle or line model made it this far, it is valid. Add a vote for it in the appropriate Hough parameter space (circle or line, respectively).
    }

    // TODO: Cluster Parameter Spaces
    // After all iterations on a point cloud cluster, tally the votes.
    // 1. Input: The Hough parameter spaces (for lines and circles) containing accumulated votes from RANSAC iterations.
    // 2. Method:
    //    - Use a region growing approach based on nearest neighbor to group nearby votes in the parameter spaces into clusters.
    // 3. Output: Clusters in the parameter spaces, where each cluster represents a potential object in the original scene.

    // TODO: Select Model with Most Votes
    // Based on which parameter space (line or circle) contains the highest concentration of votes, decide whether to fit a box or a cylinder model.

    // TODO: Estimate 3D Shape
    // Using the parameters from the highest-vote cluster, fit the selected solid geometric primitive model type (box or cylinder) to the original 3D point cloud data.

    // TODO: Add Shape as Collision Object
    // For now, we'll always create a cylinder as a placeholder
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "cylinder_" + std::to_string(cylinder_count++);
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.35;  // height
    primitive.dimensions[1] = 0.0125; // radius

    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;
    cylinder_pose.position.x = 0.22;
    cylinder_pose.position.y = 0.12;
    cylinder_pose.position.z = 0.175;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    collision_objects.push_back(collision_object);

    // TODO: Check if there are Points Remaining for this Cluster
    // - If points remain in the point cloud cluster, go back to the top of the Inner Loop (starting with I=0) for the remaining points in the cluster.
    // - If no points remain, move to the next point cloud cluster in the vector (i.e. move to the next iteration of the Outer Loop)
  }


  return collision_objects;
}

// Helper function implementations will be added here as we develop them


