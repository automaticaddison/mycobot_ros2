/**
 * @file normals_curvature_and_rsd_estimation.cpp
 * @brief Estimate normal vectors, curvature values, and RSD values for each point in the point cloud.
 *
 * This file contains a function to estimate normal vectors, curvature values,
 * and Radius-based Surface Descriptor (RSD) values for each point in a given point cloud.
 * It uses the PCL library for point cloud processing.
 *
 * @author Addison Sears-Collins 
 * @date September 21, 2024
 */

#include "hello_mtc_with_perception/normals_curvature_and_rsd_estimation.h"

// Define a logging function
void LOG_INFO(const std::string& msg) {
  std::cout << "[INFO] " << msg << std::endl;
}

pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr 
estimateNormalsCurvatureAndRSD(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    int k_neighbors,  
    double max_plane_error,
    int max_iterations,  
    int min_boundary_neighbors,
    double rsd_radius) {
  LOG_INFO("Starting normal, curvature, and RSD estimation. Input cloud size: " + std::to_string(input_cloud->size()) + " points");

  // Output: Point Cloud with Normal Vectors, Curvature Values, and RSD Values
  auto output_cloud = std::make_shared<pcl::PointCloud<PointXYZRGBNormalRSD>>();
  
  // Create a KdTree for efficient neighbor searches
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
  tree->setInputCloud(input_cloud);

  // Process: For each point p in the cloud
  for (size_t i = 0; i < input_cloud->size(); ++i) {
    // Step 1: Find neighbors
    std::vector<int> indices;
    std::vector<float> distances;
    tree->nearestKSearch(input_cloud->points[i], k_neighbors, indices, distances);

    // Simple boundary point check
    bool is_boundary = (indices.size() < static_cast<size_t>(k_neighbors));

    // Adjust neighborhood size for boundary points
    int used_k = is_boundary ? std::min(static_cast<int>(indices.size()), min_boundary_neighbors) : k_neighbors;

    // Use only the first 'used_k' neighbors
    indices.resize(used_k);
    distances.resize(used_k);

    // Extract neighborhood points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*input_cloud, indices, *neighborhood);

    // Step 2 & 3: Perform Principal Component Analysis (PCA) on this neighborhood
    pcl::PCA<pcl::PointXYZRGB> initial_pca;
    initial_pca.setInputCloud(neighborhood);

    // The eigenvector corresponding to the smallest eigenvalue is an initial normal estimate
    Eigen::Vector3f initial_normal = initial_pca.getEigenVectors().col(2);

    // Step 4: Use Maximum Likelihood Estimation SAmple Consensus (MLESAC) to refine the local plane estimate
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
        model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(neighborhood));
    pcl::MaximumLikelihoodSampleConsensus<pcl::PointXYZRGB> mlesac(model);
    mlesac.setDistanceThreshold(max_plane_error);
    mlesac.setMaxIterations(max_iterations);
    
    Eigen::VectorXf coefficients;
    std::vector<int> inliers;
    if (mlesac.computeModel()) {
      mlesac.getModelCoefficients(coefficients);
      mlesac.getInliers(inliers);
    } else {
      // If MLESAC fails, use the initial normal
      coefficients = Eigen::VectorXf(4);
      coefficients.head<3>() = initial_normal;
      coefficients(3) = -initial_normal.dot(initial_pca.getMean().head<3>());
    }

    // Step 5: Compute a weighted covariance matrix
    Eigen::Matrix3f weighted_covariance = Eigen::Matrix3f::Zero();
    Eigen::Vector3f weighted_mean = Eigen::Vector3f::Zero();
    float total_weight = 0.0f;

    // Compute mean distance μ
    float mu = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();

    // Compute weighted mean
    for (size_t j = 0; j < indices.size(); ++j) {
      Eigen::Vector3f pt = neighborhood->points[j].getVector3fMap();
      // Assign weights to neighbor points based on their distance to the estimated local plane
      float weight = (std::find(inliers.begin(), inliers.end(), j) != inliers.end()) ? 
                     1.0f : std::exp(-(distances[j] * distances[j]) / (mu * mu));
      weighted_mean += weight * pt;
      total_weight += weight;
    }
    weighted_mean /= total_weight;

    // Compute weighted covariance
    for (size_t j = 0; j < indices.size(); ++j) {
      Eigen::Vector3f pt = neighborhood->points[j].getVector3fMap();
      Eigen::Vector3f pt_mean = pt - weighted_mean;
      float weight = (std::find(inliers.begin(), inliers.end(), j) != inliers.end()) ? 
                     1.0f : std::exp(-(distances[j] * distances[j]) / (mu * mu));
      weighted_covariance += weight * pt_mean * pt_mean.transpose();
    }
    weighted_covariance /= total_weight;

    // Step 6: Perform PCA on this weighted covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(weighted_covariance);
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();

    // The eigenvector corresponding to the smallest eigenvalue is the final normal estimate
    Eigen::Vector3f final_normal = eigenvectors.col(0);

    // Step 7: Compute curvature values
    // Sort eigenvalues in ascending order
    std::sort(eigenvalues.data(), eigenvalues.data() + 3);
    // Compute curvature using the formula: γp = λ₀ / (λ₀ + λ₁ + λ₂) where λ₀ ≤ λ₁ ≤ λ₂
    float curvature = eigenvalues(0) / eigenvalues.sum();

    // Step 8: Estimate RSD values
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->points.resize(1);
    normals->points[0].normal_x = final_normal(0);
    normals->points[0].normal_y = final_normal(1);
    normals->points[0].normal_z = final_normal(2);

    pcl::RSDEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
    rsd.setInputCloud(input_cloud);
    rsd.setInputNormals(normals);
    rsd.setSearchMethod(tree);
    rsd.setRadiusSearch(rsd_radius);

    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsd_features(new pcl::PointCloud<pcl::PrincipalRadiiRSD>);
    rsd.compute(*rsd_features);

    // Step 9: Output creation
    PointXYZRGBNormalRSD point;
    // The original XYZ coordinates
    point.x = input_cloud->points[i].x;
    point.y = input_cloud->points[i].y;
    point.z = input_cloud->points[i].z;
    // The RGB color information
    point.r = input_cloud->points[i].r;
    point.g = input_cloud->points[i].g;
    point.b = input_cloud->points[i].b;
    // The estimated normal vector for each point
    point.normal_x = final_normal(0);
    point.normal_y = final_normal(1);
    point.normal_z = final_normal(2);
    // The estimated curvature value for each point
    point.curvature = curvature;
    // The estimated RSD values
    point.r_min = rsd_features->points[0].r_min;
    point.r_max = rsd_features->points[0].r_max;
    output_cloud->points.push_back(point);
  }

  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;
  output_cloud->is_dense = true;

  // Log sample of output data structure
  const int sample_size = std::min(5, static_cast<int>(output_cloud->size()));
  std::stringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "Sample of output data structure (showing first " << sample_size << " points):\n";
  ss << "Point\tX\tY\tZ\tR\tG\tB\tNormal X\tNormal Y\tNormal Z\tCurvature\tRSD Min\tRSD Max\n";
  
  for (int i = 0; i < sample_size; ++i) {
    const auto& point = output_cloud->points[i];
    ss << i << "\t"
       << point.x << "\t" << point.y << "\t" << point.z << "\t"
       << static_cast<int>(point.r) << "\t" << static_cast<int>(point.g) << "\t" << static_cast<int>(point.b) << "\t"
       << point.normal_x << "\t" << point.normal_y << "\t" << point.normal_z << "\t"
       << point.curvature << "\t"
       << point.r_min << "\t" << point.r_max << "\n";
  }
  
  LOG_INFO("Normal, curvature, and RSD estimation completed. Output cloud size: " + std::to_string(output_cloud->size()) + " points");
  LOG_INFO("Sample output:\n" + ss.str());

  return output_cloud;
}

