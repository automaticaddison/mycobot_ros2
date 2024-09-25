#include "hello_mtc_with_perception/object_segmentation.h"

// Line fitting helpers
// Computes the coefficients (a, b, c) of a 2D line in the form a*x + b*y + c = 0 
// that passes through two points p1 and p2.
Eigen::Vector3f fitLine(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {
  Eigen::Vector3f line;
  line[0] = p2.y() - p1.y();  // a
  line[1] = p1.x() - p2.x();  // b
  line[2] = p2.x() * p1.y() - p1.x() * p2.y();  // c
  line.normalize();
  return line;
}
// Calculates the perpendicular distance from a given point to a line 
// defined by coefficients (a, b, c).
float distanceToLine(const Eigen::Vector2f& point, const Eigen::Vector3f& line) {
  return std::abs(line[0] * point.x() + line[1] * point.y() + line[2]) / 
         std::sqrt(line[0] * line[0] + line[1] * line[1]);
}

// RANSAC for 2D line fitting
// Robustly fit a line to a set of 2D points, even in the presence of outliers.
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitLineRANSAC(
    const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
    double ransac_distance_threshold,
    int ransac_max_iterations) {
  
  pcl::PointIndices::Ptr best_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr best_coefficients(new pcl::ModelCoefficients);
  best_coefficients->values.resize(3);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, cloud->points.size() - 1);

  for (int iter = 0; iter < ransac_max_iterations; ++iter) {
    // Randomly select two different points from the cluster to define a candidate line.
    int idx1 = dis(gen);
    int idx2 = dis(gen);
    if (idx1 == idx2) continue;

    Eigen::Vector2f p1(cloud->points[idx1].x, cloud->points[idx1].y);
    Eigen::Vector2f p2(cloud->points[idx2].x, cloud->points[idx2].y);
    Eigen::Vector3f line = fitLine(p1, p2);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector2f pt(cloud->points[i].x, cloud->points[i].y);
      if (distanceToLine(pt, line) < ransac_distance_threshold) {
        inliers->indices.push_back(i);
      }
    }

    // Keep track of the candidate line with the most inliers.
    if (inliers->indices.size() > best_inliers->indices.size()) {
      best_inliers = inliers;
      best_coefficients->values[0] = line[0];
      best_coefficients->values[1] = line[1];
      best_coefficients->values[2] = line[2];
    }
  }

  // Returns the indices of the inlier points and the best line coefficients found.
  return std::make_tuple(best_inliers, best_coefficients);
}

// Calculate the center (h, k) and radius r of a circle passing through three points p1, p2, and p3.
Eigen::Vector3f fitCircle(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& p3) {
  Eigen::Matrix3f A;
  Eigen::Vector3f b;
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector2f& p = (i == 0) ? p1 : ((i == 1) ? p2 : p3);
    A.row(i) << 2*p.x(), 2*p.y(), 1;
    b(i) = p.x()*p.x() + p.y()*p.y();
  }
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
  float radius = std::sqrt(x(0)*x(0) + x(1)*x(1) + x(2));
  return Eigen::Vector3f(x(0), x(1), radius);
}
// Calculate the distance from a point to the circumference of a circle.
float distanceToCircle(const Eigen::Vector2f& point, const Eigen::Vector3f& circle) {
  return std::abs((point - circle.head<2>()).norm() - circle[2]);
}

// RANSAC for 2D circle fitting
// Robustly fit a circle to a set of 2D points.
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitCircleRANSAC(
    const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
    double ransac_distance_threshold,
    int ransac_max_iterations,
    double max_allowable_radius) {
  
  pcl::PointIndices::Ptr best_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr best_coefficients(new pcl::ModelCoefficients);
  best_coefficients->values.resize(3);
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, cloud->points.size() - 1);

  for (int iter = 0; iter < ransac_max_iterations; ++iter) {
    // Randomly select three different points to define a candidate circle.
    int idx1 = dis(gen);
    int idx2 = dis(gen);
    int idx3 = dis(gen);
    if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) continue;

    Eigen::Vector2f p1(cloud->points[idx1].x, cloud->points[idx1].y);
    Eigen::Vector2f p2(cloud->points[idx2].x, cloud->points[idx2].y);
    Eigen::Vector2f p3(cloud->points[idx3].x, cloud->points[idx3].y);
    Eigen::Vector3f circle = fitCircle(p1, p2, p3);
    
    // Sanity check: ignore circles with unreasonable radii
    if (circle[2] > max_allowable_radius || circle[2] <= 0) {
      continue;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector2f pt(cloud->points[i].x, cloud->points[i].y);
      if (distanceToCircle(pt, circle) < ransac_distance_threshold) {
        inliers->indices.push_back(i);
      }
    }

    // Keep track of the candidate circle with the most inliers.
    if (inliers->indices.size() > best_inliers->indices.size()) {
      best_inliers = inliers;
      best_coefficients->values[0] = circle[0];  // center x
      best_coefficients->values[1] = circle[1];  // center y
      best_coefficients->values[2] = circle[2];  // radius
    }
  }
  // Return the indices of the inlier points and the best circle coefficients found.
  return std::make_tuple(best_inliers, best_coefficients);
}

// Helper method to log line and circle model fitting results
void logModelResults(const std::string& modelType,
                     const pcl::ModelCoefficients::Ptr& coefficients,
                     const pcl::PointIndices::Ptr& inliers) {
  std::ostringstream log_stream;
  if (inliers->indices.size() > 0) {
    if (modelType == "Line") {
      log_stream << "Line model: " << coefficients->values[0] << " x + "
                 << coefficients->values[1] << " y + "
                 << coefficients->values[2] << " = 0";
    } else if (modelType == "Circle") {
      log_stream << "Circle model: center (" << coefficients->values[0] << ", "
                 << coefficients->values[1] << "), radius " << coefficients->values[2];
    }
    LOG_INFO(log_stream.str());

    log_stream.str("");
    log_stream << modelType << " model inliers: " << inliers->indices.size();
    LOG_INFO(log_stream.str());
  } else {
    LOG_INFO("Could not estimate a " + modelType + " model for the given dataset.");
  }
}

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
    int ransac_max_iterations) {
  
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

    // Create a copy of the original projected cloud
    pcl::PointCloud<pcl::PointXY>::Ptr original_projected_cloud_copy(new pcl::PointCloud<pcl::PointXY>);
    pcl::copyPointCloud(*projected_cloud, *original_projected_cloud_copy);
    
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

    /****************************************************
     *                                                  *
     *                Inner Loop                        *
     *                                                  *
     ***************************************************/  
    for (int i = 0; i < num_iterations; ++i) {
      bool test_single_iteration = true;  // Add this flag for testing TODO

      /****************************************************
       *                                                  *
       *                RANSAC Model Fitting              *
       *                                                  *
       ***************************************************/ 
      while (static_cast<int>(projected_cloud->points.size()) > inlier_threshold) {
      
        // Line Fitting
        // - Use RANSAC to fit a 2D line to the projected points. This is done to identify potential box-like objects (lines).
        auto [line_inliers, line_coefficients] = fitLineRANSAC(projected_cloud, 
                                                           ransac_distance_threshold, 
                                                           ransac_max_iterations);


        // Circle Fitting
        // - Use RANSAC to fit a 2D circle to the projected points. This is done to identify cylindrical-like objects (cylinders)
        auto [circle_inliers, circle_coefficients] = fitCircleRANSAC(projected_cloud, 
                                                                 ransac_distance_threshold, 
                                                                 ransac_max_iterations,
                                                                 hough_max_radius);

        // Log the results
        logModelResults("Line", line_coefficients, line_inliers);
        logModelResults("Circle", circle_coefficients, circle_inliers);

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

         // TODO: Create a data structure called inliers_to_remove that to use in the next step to track indices of inliers of valid models

        // TODO: Add Model to the Hough Space (pcl::Hough3DGrouping)
        // If a circle or line model made it this far, it is valid. 
        // If a circle model is valid:
        // - Add a vote for it in the circle Hough parameter space 
        // - Store the indices of the inliers for this circle model. 
        // If a line model is valid:
        // - Add a vote for it in the line Hough parameter space 
        // - Store the indices of the inliers for this line model

        // TODO: Remove duplicate inliers that appear in both the circle and line inlier list (if applicable)

        // TODO: Create a new point cloud called cloud_without_inliers that is the projected cloud with the valid model inliers removed
        //        -- Update the projected cloud for the next interation of the while loop

        // Otherwise, if no valid models were found, exit this while loop
        if (test_single_iteration) {
          LOG_INFO("Breaking after first iteration for testing purposes.");
          break;
        }
      }
      // Restore the full point cloud for the next iteration I
      pcl::copyPointCloud(*original_projected_cloud_copy, *projected_cloud);       
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
    // Remove the detected object from the 3D cluster
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
    // - If points remain in the point cloud cluster, create a 3D bounding box for the residual points and add it as a collision object.
    // - If no points remain, move to the next point cloud cluster in the vector (i.e. move to the next iteration of the Outer Loop)
  }


  return collision_objects;
}


