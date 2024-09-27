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
    int ransac_max_iterations,
    const std::unordered_map<size_t, size_t>& projection_map) {
  
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
        inliers->indices.push_back(projection_map.at(i));  // Use the mapping here
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
    double max_allowable_radius,
    const std::unordered_map<size_t, size_t>& projection_map) {
  
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
        inliers->indices.push_back(projection_map.at(i));  // Use the mapping here
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
      LOG_INFO("");
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
    double circle_cluster_tolerance) {
  pcl::PointIndices::Ptr filtered_inliers(new pcl::PointIndices);

  // Step 1: Euclidean Clustering
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& idx : circle_inliers->indices) {
    size_t original_idx = projection_map.at(idx);
    const auto& point = original_cloud->points[original_idx];
    inlier_cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
  }
  inlier_cloud->width = inlier_cloud->points.size();
  inlier_cloud->height = 1;
  inlier_cloud->is_dense = true;

  LOG_INFO("- Circle inlier cloud size before filtering: " + std::to_string(inlier_cloud->points.size()));

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(inlier_cloud);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(circle_cluster_tolerance);
  ec.setMinClusterSize(circle_min_cluster_size);
  ec.setMaxClusterSize(inlier_cloud->points.size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(inlier_cloud);
  ec.extract(clusters);

  LOG_INFO("- Circle inliers clusters found: " + std::to_string(clusters.size()));

  if (clusters.empty()) {
    LOG_INFO("- No clusters found. Consider adjusting clustering parameters.");
    return filtered_inliers;
  }

  if (clusters.size() > static_cast<size_t>(circle_max_clusters)) {
    LOG_INFO("- Rejecting circle: number of clusters (" + std::to_string(clusters.size()) + 
             ") exceeds maximum allowed (" + std::to_string(circle_max_clusters) + ")");
    return filtered_inliers;
  }

  // Step 2: Height Consistency Check (only if we have exactly two clusters)
  if (clusters.size() == 2) {
    double max_height_1 = -std::numeric_limits<double>::max();
    double max_height_2 = -std::numeric_limits<double>::max();

    // Find the maximum height of points in the first cluster
    for (const auto& idx : clusters[0].indices) {
      size_t original_idx = projection_map.at(circle_inliers->indices[idx]);
      max_height_1 = std::max(max_height_1, static_cast<double>(original_cloud->points[original_idx].z));
    }

    // Find the maximum height of points in the second cluster
    for (const auto& idx : clusters[1].indices) {
      size_t original_idx = projection_map.at(circle_inliers->indices[idx]);
      max_height_2 = std::max(max_height_2, static_cast<double>(original_cloud->points[original_idx].z));
    }

    // Check if the height difference between clusters exceeds the tolerance
    if (std::abs(max_height_1 - max_height_2) > circle_height_tolerance) {
      LOG_INFO("- Rejecting circle: height difference between clusters (" + 
               std::to_string(std::abs(max_height_1 - max_height_2)) + 
               ") exceeds tolerance (" + std::to_string(circle_height_tolerance) + ")");
      return filtered_inliers; // Return empty PointIndices if height difference is too large
    }
  }

  // Extract circle parameters
  double circle_radius = circle_coefficients->values[2];
  Eigen::Vector3f circle_center(circle_coefficients->values[0], circle_coefficients->values[1], 0);

  // Step 3: Curvature, RSD, and Normal Filtering
  for (const auto& idx : circle_inliers->indices) {
    size_t original_idx = projection_map.at(idx);
    const auto& point = original_cloud->points[original_idx];
    
    // Curvature Filtering: Skip points with low curvature
    if (point.curvature < circle_curvature_threshold) continue;

    // RSD Filtering: Skip points where the minimum RSD value differs too much from the circle radius
    if (std::abs(point.r_min - circle_radius) > circle_radius_tolerance) continue;

    // Surface Normal Filtering
    Eigen::Vector3f point_vector(point.x - circle_center.x(), point.y - circle_center.y(), 0);
    Eigen::Vector3f normal_vector(point.normal_x, point.normal_y, 0);
    point_vector.normalize();
    normal_vector.normalize();

    float dot_product = std::abs(point_vector.dot(normal_vector));
    float angle = std::acos(dot_product);

    // Check if the minimum angle between the normal and the radial vector exceeds the threshold
    if (std::min(angle, static_cast<float>(M_PI) - angle) > circle_normal_angle_threshold) continue;

    // If all checks pass, add to filtered inliers
    filtered_inliers->indices.push_back(original_idx);
  }

  // Log the number of inliers remaining after filtering
  LOG_INFO("- Circle inlier cloud size after filtering: " + 
           std::to_string(filtered_inliers->indices.size()));

  return filtered_inliers;
}

pcl::PointIndices::Ptr filterLineInliers(
  const pcl::PointIndices::Ptr& line_inliers,
  const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& original_cloud,
  [[maybe_unused]] const pcl::ModelCoefficients::Ptr& line_coefficients,
  const std::unordered_map<size_t, size_t>& projection_map,
  int line_min_cluster_size,
  int line_max_clusters,
  double line_curvature_threshold,
  [[maybe_unused]] double line_normal_angle_threshold,
  double line_cluster_tolerance) {
  
  pcl::PointIndices::Ptr filtered_inliers(new pcl::PointIndices);

  // Step 1: Euclidean Clustering
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& idx : line_inliers->indices) {
    size_t original_idx = projection_map.at(idx);
    const auto& point = original_cloud->points[original_idx];
    inlier_cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
  }
  inlier_cloud->width = inlier_cloud->points.size();
  inlier_cloud->height = 1;
  inlier_cloud->is_dense = true;

  LOG_INFO("- Line inlier cloud size before filtering: " + std::to_string(inlier_cloud->points.size()));

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(inlier_cloud);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(line_cluster_tolerance);
  ec.setMinClusterSize(line_min_cluster_size);
  ec.setMaxClusterSize(inlier_cloud->points.size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(inlier_cloud);
  ec.extract(clusters);

  LOG_INFO("- Line inliers clusters found: " + std::to_string(clusters.size()));

  // Reject if number of clusters exceeds the maximum allowed
  if (clusters.size() > static_cast<size_t>(line_max_clusters)) {
    LOG_INFO("- Rejecting line: number of clusters (" + std::to_string(clusters.size()) +
             ") exceeds maximum allowed (" + std::to_string(line_max_clusters) + ")");
    return filtered_inliers;
  }

  // Step 2: Curvature Filtering
  int curvature_filtered = 0;

  for (const auto& idx : line_inliers->indices) {
    size_t original_idx = projection_map.at(idx);
    const auto& point = original_cloud->points[original_idx];

    // Curvature Filtering: Skip points with high curvature
    if (point.curvature > line_curvature_threshold) {
      curvature_filtered++;
      continue;
    }

    // If curvature check passes, add to filtered inliers
    filtered_inliers->indices.push_back(original_idx);
  }

  // Log the number of inliers remaining after filtering
  LOG_INFO("- Line inlier cloud size after filtering: " + std::to_string(filtered_inliers->indices.size()));

  return filtered_inliers;
}

// Helper function to check if two bins are neighbors in the Hough space
static bool areBinsNeighbors(const std::vector<int>& bin1, const std::vector<int>& bin2, int maxDistance = 1) {
  if (bin1.size() != bin2.size()) return false;
  for (size_t i = 0; i < bin1.size(); ++i) {
    if (std::abs(bin1[i] - bin2[i]) > maxDistance) return false;
  }
  return true;
}

std::vector<HoughBin> clusterLineHoughSpace(
    const Eigen::MatrixXi& houghSpaceLine,
    double houghAngleStep,
    double houghRhoStep) {
  
  std::vector<HoughBin> clusters;
  std::vector<std::tuple<int, int, int>> voteBins; // (votes, i, j)
  
  // Collect all bins with non-zero votes
  for (int i = 0; i < houghSpaceLine.rows(); ++i) {
    for (int j = 0; j < houghSpaceLine.cols(); ++j) {
      if (houghSpaceLine(i, j) > 0) {
        voteBins.emplace_back(houghSpaceLine(i, j), i, j);
      }
    }
  }
  
  // Sort bins by vote count in descending order
  std::sort(voteBins.begin(), voteBins.end(),
            [](const auto& a, const auto& b) { return std::get<0>(a) > std::get<0>(b); });
  
  std::vector<bool> processed(voteBins.size(), false);
  
  for (size_t k = 0; k < voteBins.size(); ++k) {
    if (processed[k]) continue;
    
    int votes, i, j;
    std::tie(votes, i, j) = voteBins[k];
    
    HoughBin newCluster;
    newCluster.indices = {i, j};
    newCluster.votes = votes;
    newCluster.parameters = {
      j * houghRhoStep,  // rho (directly from bin index)
      i * houghAngleStep  // theta
    };
    
    processed[k] = true;
    
    // Check for neighbors and merge
    for (size_t l = k + 1; l < voteBins.size(); ++l) {
      if (processed[l]) continue;
      
      int neighborVotes, ni, nj;
      std::tie(neighborVotes, ni, nj) = voteBins[l];
      
      if (areBinsNeighbors({i, j}, {ni, nj})) {
        newCluster.votes += neighborVotes;
        // Update parameters (weighted average)
        double totalVotes = newCluster.votes;
        newCluster.parameters[0] = (newCluster.parameters[0] * (totalVotes - neighborVotes) + 
                                    nj * houghRhoStep * neighborVotes) / totalVotes;
        newCluster.parameters[1] = (newCluster.parameters[1] * (totalVotes - neighborVotes) + 
                                    ni * houghAngleStep * neighborVotes) / totalVotes;
        processed[l] = true;
      }
    }
    
    clusters.push_back(newCluster);
  }
  
  return clusters;
}

std::vector<HoughBin> clusterCircleHoughSpace(
    const Eigen::Tensor<int, 3>& houghSpaceCircle,
    double houghCenterXStep,
    double houghCenterYStep,
    double houghRadiusStep,
    double minPt_x,
    double minPt_y) {
  
  std::vector<HoughBin> clusters;
  std::vector<std::tuple<int, int, int, int>> voteBins; // (votes, i, j, k)
  
  // Collect all bins with non-zero votes
  for (int i = 0; i < houghSpaceCircle.dimension(0); ++i) {
    for (int j = 0; j < houghSpaceCircle.dimension(1); ++j) {
      for (int k = 0; k < houghSpaceCircle.dimension(2); ++k) {
        if (houghSpaceCircle(i, j, k) > 0) {
          voteBins.emplace_back(houghSpaceCircle(i, j, k), i, j, k);
        }
      }
    }
  }
  
  // Sort bins by vote count in descending order
  std::sort(voteBins.begin(), voteBins.end(),
            [](const auto& a, const auto& b) { return std::get<0>(a) > std::get<0>(b); });
  
  std::vector<bool> processed(voteBins.size(), false);
  
  for (size_t m = 0; m < voteBins.size(); ++m) {
    if (processed[m]) continue;
    
    int votes, i, j, k;
    std::tie(votes, i, j, k) = voteBins[m];
    
    HoughBin newCluster;
    newCluster.indices = {i, j, k};
    newCluster.votes = votes;
    newCluster.parameters = {minPt_x + i * houghCenterXStep, minPt_y + j * houghCenterYStep, k * houghRadiusStep};
    
    processed[m] = true;
    
    // Check for neighbors and merge
    for (size_t n = m + 1; n < voteBins.size(); ++n) {
      if (processed[n]) continue;
      
      int neighborVotes, ni, nj, nk;
      std::tie(neighborVotes, ni, nj, nk) = voteBins[n];
      
      if (areBinsNeighbors({i, j, k}, {ni, nj, nk})) {
        newCluster.votes += neighborVotes;
        // Update parameters (weighted average)
        double totalVotes = newCluster.votes;
        newCluster.parameters[0] = (newCluster.parameters[0] * (totalVotes - neighborVotes) + (minPt_x + ni * houghCenterXStep) * neighborVotes) / totalVotes;
        newCluster.parameters[1] = (newCluster.parameters[1] * (totalVotes - neighborVotes) + (minPt_y + nj * houghCenterYStep) * neighborVotes) / totalVotes;
        newCluster.parameters[2] = (newCluster.parameters[2] * (totalVotes - neighborVotes) + nk * houghRadiusStep * neighborVotes) / totalVotes;
        processed[n] = true;
      }
    }
    
    clusters.push_back(newCluster);
  }
  
  return clusters;
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
    double line_cluster_tolerance) {
  
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
    log_stream.str(""); // Clear the stream
    log_stream << "\n\n\n"  // Add three blank lines
           << "***********************************************************************************************\n"
           << "Successfully projected " << cluster->points.size()
           << " 3D points onto the z=0 plane. Resulting 2D cloud has "
           << projected_cloud->points.size() << " points.\n"
           << "***********************************************************************************************";
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
      << "    - ρ (distance): " << hough_rho_bins << " bins (range: 0 to " << hough_max_distance << " m, step: " << hough_rho_step << " m)\n"
      << "  Circle Hough Space (3D):\n"
      << "    - Dimensions: " << hough_center_bins << " x " << hough_center_bins << " x " << hough_radius_bins << "\n"
      << "    - Center X: " << hough_center_bins << " bins (range: " << min_pt.x << " to " << max_pt.x << " m, step: " << hough_center_x_step << " m)\n"
      << "    - Center Y: " << hough_center_bins << " bins (range: " << min_pt.y << " to " << max_pt.y << " m, step: " << hough_center_y_step << " m)\n"
      << "    - Radius: " << hough_radius_bins << " bins (range: 0 to " << hough_max_radius << " m, step: " << hough_radius_step << " m)\n"
      << "  Projected 2D Space Dimensions:\n"
      << "    - X-range: " << projected_x_range << " m\n"
      << "    - Y-range: " << projected_y_range << " m\n"
      << "    - Diagonal: " << hough_max_distance << " m\n";
    LOG_INFO(log_stream.str());

    /****************************************************
     *                                                  *
     *                Inner Loop                        *
     *                                                  *
     ***************************************************/  
    for (int i = 0; i < num_iterations; ++i) {

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
                                                           ransac_max_iterations,
                                                           projection_map);


        // Circle Fitting
        // - Use RANSAC to fit a 2D circle to the projected points. This is done to identify cylindrical-like objects (cylinders)
        auto [circle_inliers, circle_coefficients] = fitCircleRANSAC(projected_cloud, 
                                                                 ransac_distance_threshold, 
                                                                 ransac_max_iterations,
                                                                 hough_max_radius,
                                                                 projection_map);      

        /****************************************************
         *                                                  *
         *                Filter Inliers                    *
         *                                                  *
         ***************************************************/ 
        logModelResults("Circle", circle_coefficients, circle_inliers);

        // Circle Filtering
        pcl::PointIndices::Ptr filtered_circle_inliers = filterCircleInliers(
          circle_inliers,
          cluster,
          circle_coefficients,
          projection_map,
          circle_min_cluster_size,
          circle_max_clusters,
          circle_height_tolerance,
          circle_curvature_threshold,
          circle_radius_tolerance,
          circle_normal_angle_threshold,
          circle_cluster_tolerance);

        logModelResults("Line", line_coefficients, line_inliers);

        // Line Filtering
        pcl::PointIndices::Ptr filtered_line_inliers = filterLineInliers(
          line_inliers,
          cluster,
          line_coefficients,
          projection_map,
          line_min_cluster_size,
          line_max_clusters,
          line_curvature_threshold,
          line_normal_angle_threshold,
          line_cluster_tolerance);

        /****************************************************
         *                                                  *
         *                Model Validation                  *
         *                                                  *
         ***************************************************/ 
        // For the circle and line models, check how many inlier points remain.
        // - If the number of remaining inliers for the model exceeds the inlier_threshold (i.e. 100 points):
        //   - The model is considered valid.
        //   - Add the model (circle or line depending on the inlier group) to the valid models list. It will be added to the Hough parameter space in the next step.
        // - If the number of remaining inliers is below the threshold:
        //   - The model (circle or line as applicable) is rejected.
        std::vector<ValidModel> valid_models;

        // Validate Circle Model
        if (static_cast<int>(filtered_circle_inliers->indices.size()) > inlier_threshold) {
          ValidModel circle_model;
          circle_model.type = "circle";
          circle_model.parameters = {
            circle_coefficients->values[0],  // center_x
            circle_coefficients->values[1],  // center_y
            circle_coefficients->values[2]   // radius
          };
          circle_model.inlier_indices = filtered_circle_inliers->indices;
          valid_models.push_back(circle_model);
          LOG_INFO("");
          LOG_INFO("=== Circle Model Validated ===");
          LOG_INFO("  Inliers: " + std::to_string(circle_model.inlier_indices.size()));
          LOG_INFO("  Center: (" + std::to_string(circle_model.parameters[0]) + ", " + 
                   std::to_string(circle_model.parameters[1]) + ")");
          LOG_INFO("  Radius: " + std::to_string(circle_model.parameters[2]));
        } else {
          LOG_INFO("");
          LOG_INFO("=== Circle Model Rejected ===");
          LOG_INFO("  Inliers: " + std::to_string(filtered_circle_inliers->indices.size()) + 
             " (Threshold: " + std::to_string(inlier_threshold) + ")");
        }

        // Validate Line Model
        if (static_cast<int>(filtered_line_inliers->indices.size()) > inlier_threshold) {
          ValidModel line_model;
          line_model.type = "line";
          // Convert line coefficients to rho and theta form
          double a = line_coefficients->values[0];
          double b = line_coefficients->values[1];
          double c = line_coefficients->values[2];
          double rho = std::abs(c) / std::sqrt(a*a + b*b);
          double theta = std::atan2(b, a);
          if (theta < 0) theta += M_PI;  // Ensure theta is in [0, π)
          line_model.parameters = {rho, theta};
          line_model.inlier_indices = filtered_line_inliers->indices;
          valid_models.push_back(line_model);
          LOG_INFO("");
          LOG_INFO("=== Line Model Validated ===");
          LOG_INFO("  Inliers: " + std::to_string(line_model.inlier_indices.size()));
          LOG_INFO("  Rho: " + std::to_string(rho));
          LOG_INFO("  Theta: " + std::to_string(theta) + " radians");
        } else {
          LOG_INFO("");
          LOG_INFO("=== Line Model Rejected ===");
          LOG_INFO("  Inliers: " + std::to_string(filtered_line_inliers->indices.size()) + 
                   " (Threshold: " + std::to_string(inlier_threshold) + ")");
        }
        LOG_INFO("");
        LOG_INFO("=== Model Validation Summary ===");
        LOG_INFO("  Total valid models: " + std::to_string(valid_models.size()));
        LOG_INFO("  Circle models: " + std::to_string(std::count_if(valid_models.begin(), valid_models.end(), 
                                      [](const ValidModel& m) { return m.type == "circle"; })));
        LOG_INFO("  Line models: " + std::to_string(std::count_if(valid_models.begin(), valid_models.end(), 
                                [](const ValidModel& m) { return m.type == "line"; })));
        LOG_INFO("==============================");

        // If no valid models were found, break out of this while loop
        if (valid_models.empty()) {
          break;
        }

        // Create a data structure called inliers_to_remove that contains the indices of the inliers for the valid models you just found.
        std::set<int> inliers_to_remove;
        for (const auto& model : valid_models) {
          inliers_to_remove.insert(model.inlier_indices.begin(), model.inlier_indices.end());
        }

        /****************************************************
         *                                                  *
         *      Add Valid Models to the Hough Space         *
         *                                                  *
         ***************************************************/         
        // Add valid models to the the Hough Space you created in an earlier step
        // If a circle model is valid:
        // - Add a vote for it in the circle Hough parameter space 
        // If a line model is valid:
        // - Add a vote for it in the line Hough parameter space 
        for (const auto& model : valid_models) {
          if (model.type == "circle") {
            // Calculate indices for circle Hough space
            int center_x_bin = static_cast<int>((model.parameters[0] - min_pt.x) / hough_center_x_step);
            int center_y_bin = static_cast<int>((model.parameters[1] - min_pt.y) / hough_center_y_step);
            int radius_bin = static_cast<int>(model.parameters[2] / hough_radius_step);

            // Ensure indices are within bounds
            center_x_bin = std::clamp(center_x_bin, 0, hough_center_bins - 1);
            center_y_bin = std::clamp(center_y_bin, 0, hough_center_bins - 1);
            radius_bin = std::clamp(radius_bin, 0, hough_radius_bins - 1);

            // Add vote to circle Hough space
            hough_space_circle(center_x_bin, center_y_bin, radius_bin) += 1;

          }
          else if (model.type == "line") {
            // Calculate indices for line Hough space
            double rho = model.parameters[0];
            double theta = model.parameters[1];
            
            int theta_bin = static_cast<int>(theta / hough_angle_step);
            int rho_bin = static_cast<int>(rho / hough_rho_step);

            // Ensure indices are within bounds
            theta_bin = std::clamp(theta_bin, 0, hough_angle_bins - 1);
            rho_bin = std::clamp(rho_bin, 0, hough_rho_bins - 1);

            // Add vote to line Hough space
            hough_space_line(theta_bin, rho_bin) += 1;

          }
        }

        // Create a new point cloud called cloud_without_inliers that is the projected cloud minus the inliers_to_remove 
        pcl::PointCloud<pcl::PointXY>::Ptr cloud_without_inliers(new pcl::PointCloud<pcl::PointXY>);
        for (size_t i = 0; i < projected_cloud->points.size(); ++i) {
          if (inliers_to_remove.find(i) == inliers_to_remove.end()) {
            cloud_without_inliers->points.push_back(projected_cloud->points[i]);
          }
        }
        cloud_without_inliers->width = cloud_without_inliers->points.size();
        cloud_without_inliers->height = 1;
        cloud_without_inliers->is_dense = true;

        // Update the projected_cloud (i.e. projected_cloud = cloud_without_inliers) for the next iteration of the while loop
        projected_cloud = cloud_without_inliers;

        LOG_INFO("Removed " + std::to_string(inliers_to_remove.size()) + " inliers. New projected cloud size: " + std::to_string(projected_cloud->points.size()));
        LOG_INFO("");  
      }
      // Log the reason for exiting the while loop
      if (static_cast<int>(projected_cloud->points.size()) <= inlier_threshold) {
        LOG_INFO("Exiting RANSAC loop: Insufficient points remain. Points left: " + 
                 std::to_string(projected_cloud->points.size()) + 
                 ", Threshold: " + std::to_string(inlier_threshold));
      } else {
        LOG_INFO("Exiting RANSAC loop: No more valid models found.");
      }
      
      // Restore the full point cloud for the next iteration
      pcl::copyPointCloud(*original_projected_cloud_copy, *projected_cloud);
      LOG_INFO("Restored full point cloud for next iteration. Cloud size: " + std::to_string(projected_cloud->points.size()));
      LOG_INFO("Finished iteration " + std::to_string(i+1) + " of " + std::to_string(num_iterations));
      LOG_INFO("\n\n\n");    
    }

    /****************************************************
     *                                                  *
     *      Cluster Parameter Spaces                    *
     *                                                  *
     ***************************************************/  
    // After all iterations on a point cloud cluster, tally the votes.
    // 1. Input: 
    //      The Hough parameter spaces (for lines and circles) containing accumulated votes from RANSAC iterations.
    //      These votes represent 2D line or 2D circle models.
    //      I want to cluster bins that are close to each other to find the dominant models in the projected 2D point cloud we processed in the previous steps. 
    // 2. Method:
    //    - Use a region growing approach based on nearest neighbor to group Hough parameter space votes that neighbor each other into a single bin. 
    //    - This helps consolidate the circle and line models that are very similar.
    // 3. Output: A more simplified hough parameter space which contains bins with accumulated votes. Each bin represents a 2D circle or 2D line model.
    std::vector<HoughBin> clusteredLineModels = clusterLineHoughSpace(
        hough_space_line,
        hough_angle_step,
        hough_rho_step);

    std::vector<HoughBin> clusteredCircleModels = clusterCircleHoughSpace(
        hough_space_circle,
        hough_center_x_step,
        hough_center_y_step,
        hough_radius_step,
        min_pt.x,
        min_pt.y);

    LOG_INFO("Clustered Hough parameters:");
    LOG_INFO("  Line clusters: " + std::to_string(clusteredLineModels.size()));
    LOG_INFO("  Circle clusters: " + std::to_string(clusteredCircleModels.size()));

    // Log the top clusters
    int topClustersToLog = 4;
    LOG_INFO("Top line clusters:");
    for (int i = 0; i < std::min(topClustersToLog, static_cast<int>(clusteredLineModels.size())); ++i) {
      const auto& cluster = clusteredLineModels[i];
      log_stream.str("");
      log_stream << "  Cluster " << i + 1 << ": Votes = " << cluster.votes
                 << ", Rho = " << cluster.parameters[0]
                 << ", Theta = " << cluster.parameters[1];
      LOG_INFO(log_stream.str());
    }

    LOG_INFO("Top circle clusters:");
    for (int i = 0; i < std::min(topClustersToLog, static_cast<int>(clusteredCircleModels.size())); ++i) {
      const auto& cluster = clusteredCircleModels[i];
      log_stream.str("");
      log_stream << "  Cluster " << i + 1 << ": Votes = " << cluster.votes
                 << ", Center = (" << cluster.parameters[0] << ", " << cluster.parameters[1] << ")"
                 << ", Radius = " << cluster.parameters[2];
      LOG_INFO(log_stream.str());
    }

    // TODO: Select Models with the Most Votes
    // Identify the top 4 vote-getting models in the Hough parameter spaces (doesn't matter whether they are line or circle models...we just want to see which models got the most votes)
    // If the model that received the highest vote count is a circle model:
    //   - Reject all other circle and all line models in the hough parameter space. 
    //   - We will use this circle model to fit a cylinder to the original 3D point cloud cluster in the next step (Estimate 3D Shape)
    //   - Take note of the parameters of this highest-vote-count circle model because we will need it in the Estimate 3D Shape step. 
    // If the model that received the highest vote count is a line model:
    //   - Discard all circle models in the hough parameter space. 
    //   - We will use this line model to fit a box to the original 3D point cloud cluster in the Estimate 3D Shape step. 
    //   - Keep the next (up to 3 additional) vote-getting line models. Store these line models because we will use them to fit a box to the original 3D point cloud cluster in the next step (Estimate 3D Shape). These 4 lines together will be the edges of our box in the next step.

    // TODO: Estimate 3D Shape
    // If we need to fit a cylinder to the 3D point cloud cluster:
    // - The center of the circle is the (x, y) position of the cylinder.
    // - The base of the cylinder is the minimum z value among the 3D point cloud cluster.
    // - The top of the cylinder is the maximum z value among the 3D point cloud cluster.
    // - The radius of the cylinder is the radius of the circle. 
    // If we need to fit a box to the 3D point cloud cluster:
    // - Use the (up to 4) 2D line models to define the boundaries of the box. Each line represents an edge of the box ((i.e. as seen from above))
    // - The height of the box (i.e. along the z dimension) must be equal to the the maximum z value in the 3D point cloud cluster
    // - Use the inlier points on each line model to determine the width (x dimensions) and length (y dimensions) of each side (you can't assume the box sides are aligned with the x and y axes.
    // - Use the lines to determine the center of the box footprint on the 2D z=0 plane.
    // - If you only have 1 line model (i.e. this would occur if the point cloud only shows one side of the box, assume all sides are equal length). 
    // - If you have two or three lines, make a best estimate of box width and length from the inlier points as well as box orientation.

    // TODO: Add Shape as Collision Object
    // Create a moveit_msgs/CollisionObject for the 3D shape you created in the previous step. And add it to the std::vector<moveit_msgs::msg::CollisionObject> collision_objects
    
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

    // Go to the next point cloud cluster in the vector (i.e. move to the next iteration of the Outer Loop)
  }

  return collision_objects;
}


