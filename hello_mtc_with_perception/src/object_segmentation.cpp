#include "hello_mtc_with_perception/object_segmentation.h"

std::vector<moveit_msgs::msg::CollisionObject> segmentObjects(
    const std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>& cloud_clusters,
    int num_iterations,
    const std::string& frame_id,
    int inlier_threshold,
    int hough_angle_bins,
    int hough_rho_bins,
    int hough_radius_bins,
    int hough_center_bins) {
  
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  [[maybe_unused]] int box_count = 0;
  int cylinder_count = 0;
  
  std::ostringstream log_stream;
  
  /****************************************************
   *                                                  *
   *                Outer Loop                        *
   *                                                  *
   ***************************************************/    
  for (const auto& cluster : cloud_clusters) {
    // Silence unused variable warnings
    [[maybe_unused]] int num_iterations_copy = num_iterations;
    [[maybe_unused]] int inlier_threshold_copy = inlier_threshold;
    
    /****************************************************
     *                                                  *
     *       Project the 3D Point Cloud Cluster         *
     *        onto the 2D Surface Plane (z=0)           *
     *                                                  *
     ***************************************************/    
    // For each point (x,y,z,RGB,normal, curvature, RSDmin and RSDmax) in the 3D cluster:
    // - Project the point onto the surface plane (assumed to be z=0). This will create a point (x, y).
    // - Maintain a mapping between each 2D projected point and its original 3D point
    // - NOTE: This implementation assumes the surface plane is z=0.
    // - If a different plane is required, the projection step should be modified accordingly.
    pcl::PointCloud<pcl::PointXY>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXY>);
    std::unordered_map<size_t, size_t> projection_map;
    
    for (size_t i = 0; i < cluster->points.size(); ++i) {
      const auto& point = cluster->points[i];
      
      // Project the point onto the z=0 plane
      // This is done by taking the x and y coordinates and discarding the z coordinate
      pcl::PointXY projected_point;
      projected_point.x = point.x;  // x coordinate remains the same
      projected_point.y = point.y;  // y coordinate remains the same
      
      // Add the projected point to the new cloud
      projected_cloud->points.push_back(projected_point);

      // Maintain a mapping between 2D projected point index and original 3D point index
      // This allows us to reference back to the original 3D point when needed
      projection_map[projected_cloud->points.size() - 1] = i;
    }
    
    // Set the properties of the projected point cloud
    projected_cloud->width = projected_cloud->points.size();
    projected_cloud->height = 1;  // This is an unorganized point cloud
    projected_cloud->is_dense = true;  // Assuming no invalid (NaN, Inf) points in the projection
    
    // Log the successful projection
    log_stream.str("");  // Clear the stream
    log_stream << "Successfully projected " << cluster->points.size() 
               << " 3D points onto the z=0 plane. Resulting 2D cloud has " 
               << projected_cloud->points.size() << " points.";
    LOG_INFO(log_stream.str());
      
    /****************************************************
     *                                                  *
     *    Initialize empty 2D Hough parameter spaces    *
     *            (lines and circles)                   *
     *                                                  *
     ***************************************************/ 
    // Determine the range of the projected points
    pcl::PointXY min_pt, max_pt;
    min_pt.x = min_pt.y = std::numeric_limits<float>::max();
    max_pt.x = max_pt.y = -std::numeric_limits<float>::max();

    for (const auto& point : projected_cloud->points) {
      min_pt.x = std::min(min_pt.x, point.x);
      min_pt.y = std::min(min_pt.y, point.y);
      max_pt.x = std::max(max_pt.x, point.x);
      max_pt.y = std::max(max_pt.y, point.y);
    }
    
    // Calculate dimensions for Hough spaces
    double projected_x_range = max_pt.x - min_pt.x;
    double projected_y_range = max_pt.y - min_pt.y;
    double hough_max_distance = std::sqrt(projected_x_range * projected_x_range + projected_y_range * projected_y_range);

    // Line Hough space setup
    Eigen::MatrixXi hough_space_line = Eigen::MatrixXi::Zero(hough_angle_bins, hough_rho_bins);
    double hough_angle_step = M_PI / hough_angle_bins;
    double hough_rho_step = hough_max_distance / hough_rho_bins;
    
    // Circle Hough space setup
    Eigen::Tensor<int, 3> hough_space_circle(hough_center_bins, hough_center_bins, hough_radius_bins);
    hough_space_circle.setZero();
    
    double hough_max_radius = hough_max_distance / 2.0;
    double hough_center_x_step = projected_x_range / hough_center_bins;
    double hough_center_y_step = projected_y_range / hough_center_bins;
    double hough_radius_step = hough_max_radius / hough_radius_bins;

    // Log the initialization of Hough spaces
    log_stream.str("");
    log_stream << "Hough Parameter Spaces Initialization\n"
      << "  Line Hough Space (2D):\n"
      << "    - Dimensions: " << hough_angle_bins << " x " << hough_rho_bins << "\n"
      << "    - θ (angle): " << hough_angle_bins << " bins (range: 0 to π, step: " << hough_angle_step << " radians)\n"
      << "    - ρ (distance): " << hough_rho_bins << " bins (range: -" << hough_max_distance << " to " << hough_max_distance << " m, step: " << hough_rho_step << " m)\n"
      << "  Circle Hough Space (3D):\n"
      << "    - Dimensions: " << hough_center_bins << " x " << hough_center_bins << " x " << hough_radius_bins << "\n"
      << "    - Center X: " << hough_center_bins << " bins (range: " << min_pt.x << " to " << max_pt.x << " m, step: " << hough_center_x_step << " m)\n"
      << "    - Center Y: " << hough_center_bins << " bins (range: " << min_pt.y << " to " << max_pt.y << " m, step: " << hough_center_y_step << " m)\n"
      << "    - Radius: " << hough_radius_bins << " bins (range: 0 to " << hough_max_radius << " m, step: " << hough_radius_step << " m)\n"
      << "  Projected 2D Space Dimensions:\n"
      << "    - X-range: " << projected_x_range << " m\n"
      << "    - Y-range: " << projected_y_range << " m\n"
      << "    - Diagonal: " << hough_max_distance << " m\n"
      << "  Note: Each point in the Hough space represents a potential line or circle in the 2D projected space (z=0 plane).";
    LOG_INFO(log_stream.str());


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

    std::ostringstream log_stream;
    log_stream << "Added cylinder collision object: id=" << collision_object.id
               << ", height=" << primitive.dimensions[0]
               << ", radius=" << primitive.dimensions[1]
               << ", position=(" << cylinder_pose.position.x
               << ", " << cylinder_pose.position.y
               << ", " << cylinder_pose.position.z << ")";
    LOG_INFO(log_stream.str());

    // TODO: Check if there are Points Remaining for this Cluster
    // - If points remain in the point cloud cluster, go back to the top of the Inner Loop (starting with I=0) for the remaining points in the cluster.
    // - If no points remain, move to the next point cloud cluster in the vector (i.e. move to the next iteration of the Outer Loop)
  }


  return collision_objects;
}

// Helper function implementations will be added here as we develop them


