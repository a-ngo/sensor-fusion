// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <vector>

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filter_res,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // voxel grid point reduction - downsampling
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filter_res, filter_res, filter_res);
  sor.filter(*cloud_filtered);

  // region based filtering
  typename pcl::PointCloud<PointT>::Ptr cloud_roi(
      new pcl::PointCloud<PointT>());
  pcl::CropBox<PointT> roi_extractor(true);
  roi_extractor.setMin(minPoint);
  roi_extractor.setMax(maxPoint);
  roi_extractor.setInputCloud(cloud_filtered);
  roi_extractor.filter(*cloud_roi);

  // rm points from ego vehicle
  std::vector<int> ego_indices;

  pcl::CropBox<PointT> ego_extractor(true);
  ego_extractor.setMin(Eigen::Vector4f(-3, -1.5, -5, 1));
  ego_extractor.setMax(Eigen::Vector4f(3, 1.5, 5, 1));
  ego_extractor.setInputCloud(cloud_roi);
  ego_extractor.filter(ego_indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (int point : ego_indices) {
    inliers->indices.push_back(point);
  }

  // filtering the ego points out
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_roi);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_roi);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud_roi;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {

  typename pcl::PointCloud<PointT>::Ptr obstacleCloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(
      new pcl::PointCloud<PointT>());

  pcl::ExtractIndices<PointT> extract;

  // generate plane cloud from inliers
  for (int index : inliers->indices) {
    planeCloud->points.push_back(cloud->points[index]);
  }

  // generate obstacle cloud via filtering out plane cloud
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacleCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstacleCloud, planeCloud);
  return segResult;
}

template <typename PointT>
std::unordered_set<int>
ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
                                   int max_iterations, float distance_tol) {
  std::unordered_set<int> inliers_result;
  srand(time(NULL));

  auto max_number{cloud->points.size()};

  for (int64_t iteration{0}; iteration <= max_iterations; ++iteration) {
    std::unordered_set<int> inliers;

    // Randomly sample subset and fit line
    while (inliers.size() < 3) {
      inliers.insert(rand() % max_number);
    }

    auto itr = inliers.begin();
    PointT point_one = cloud->points[*itr];
    ++itr;
    PointT point_two = cloud->points[*itr];
    ++itr;
    PointT point_three = cloud->points[*itr];

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    x1 = point_one.x;
    y1 = point_one.y;
    z1 = point_one.z;

    x2 = point_two.x;
    y2 = point_two.y;
    z2 = point_two.z;

    x3 = point_three.x;
    y3 = point_three.y;
    z3 = point_three.z;

    // fit plane
    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1);
    float coef = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

    // Measure distance between every point and fitted plane
    for (int index{0}; index < cloud->points.size(); index++) {
      if (inliers.count(index)) {
        continue;
      }

      PointT point = cloud->points[index];
      float distance = fabs(point.x * a + point.y * b + point.z * c + d) / coef;

      if (distance <= distance_tol) {
        inliers.insert(index);
      }
    }

    // check if result was better
    if (inliers_result.size() < inliers.size()) {
      inliers_result = inliers;
    }
  }

  return inliers_result;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // pcl version
  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  // pcl::SACSegmentation<PointT> seg;
  // seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PLANE);
  // seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setMaxIterations(maxIterations);
  // seg.setDistanceThreshold(distanceThreshold);

  // seg.setInputCloud(cloud);
  // seg.segment(*inliers, *coefficients);

  // own ransac implementation
  std::unordered_set<int> indices =
      Ransac(cloud, maxIterations, distanceThreshold);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (int point : indices) {
    inliers->indices.push_back(point);
  }

  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

static void cluster_helper(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearest = tree->search(points[index], distanceTol);

	for (int idx : nearest) {
		if (!processed[idx]) {
			cluster_helper(idx, points, cluster, processed, tree, distanceTol);
		}
	}
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(
    const std::vector<std::vector<float>> &points, KdTree *tree,
    float distance_tol, int min_size, int max_size) {
  std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i < points.size()) {
		if (processed[i]) {
            i++;
            continue;
        }

		std::vector<int> cluster;
		cluster_helper(i, points, cluster, processed, tree, distance_tol);
        if (cluster.size() >= min_size && cluster.size() <= max_size) {
            clusters.push_back(cluster);
        } else {
            for (int remove_index : cluster) {
                processed[remove_index] = false;
            }
        }
        i++;
	}
	return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
    int min_size, int max_size) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // pcl version
  // // detected obstacles
  // typename pcl::search::KdTree<PointT>::Ptr tree(
  //     new pcl::search::KdTree<PointT>);
  // tree->setInputCloud(cloud);

  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<PointT> ec;
  // ec.setClusterTolerance(cluster_tolerance);
  // ec.setMinClusterSize(min_size);
  // ec.setMaxClusterSize(max_size);
  // ec.setSearchMethod(tree);
  // ec.setInputCloud(cloud);
  // ec.extract(cluster_indices);

  // for (std::vector<pcl::PointIndices>::const_iterator it =
  //          cluster_indices.begin();
  //      it != cluster_indices.end(); ++it) {
  //   typename pcl::PointCloud<PointT>::Ptr cluster(new
  //   pcl::PointCloud<PointT>); for (const auto &idx : it->indices) {
  //     cluster->push_back((*cloud)[idx]);
  //     cluster->width = cluster->points.size();
  //     cluster->height = 1;
  //     cluster->is_dense = true;
  //   }
  //   clusters.push_back(cluster);
  // }

  // get points from cloud
  std::vector<std::vector<float>> cloud_points{};
  for (auto point : cloud->points) {
    std::vector<float> cloud_point{0, 0, 0};
    cloud_point.at(0) = point.x;
    cloud_point.at(1) = point.y;
    cloud_point.at(2) = point.z;
    cloud_points.push_back(cloud_point);
  }

  // create tree
  KdTree *tree = new KdTree;
  for (int i = 0; i < cloud_points.size(); i++) {
    tree->insert(cloud_points[i], i);
  }

  // perform actual clustering
  // TODO: move min max here?
  std::vector<std::vector<int>> clusters_ids =
      euclideanCluster(cloud_points, tree, cluster_tolerance, min_size, max_size);

  // push found clusters to clusters (result)
  for (auto cluster_ids : clusters_ids) {
    typename pcl::PointCloud<PointT>::Ptr cluster(
        new pcl::PointCloud<PointT>());

    for (auto index : cluster_ids) {
      cluster->push_back((*cloud)[index]);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
    }
    clusters.push_back(cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // TODO(a-ngo): consider only xy plane

  // covariance
  Eigen::Vector4f pca_centroid;
  pcl::compute3DCentroid(*cluster, pca_centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cluster, pca_centroid, covariance);

  // eigen vectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
      covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigen_vectors_pca = eigen_solver.eigenvectors();
  eigen_vectors_pca.col(2) =
      eigen_vectors_pca.col(0).cross(eigen_vectors_pca.col(1));

  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3, 3>(0, 0) = eigen_vectors_pca.transpose();
  p2w.block<3, 1>(0, 3) =
      -1.f * (p2w.block<3, 3>(0, 0) * pca_centroid.head<3>());
  pcl::PointCloud<PointT> cPoints;
  pcl::transformPointCloud(*cluster, cPoints, p2w);

  PointT min_pt, max_pt;
  pcl::getMinMax3D(cPoints, min_pt, max_pt);
  const Eigen::Vector3f mean_diag =
      0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // final transform
  const Eigen::Quaternionf qfinal(eigen_vectors_pca);
  const Eigen::Vector3f tfinal =
      eigen_vectors_pca * mean_diag + pca_centroid.head<3>();

  BoxQ boxq;
  boxq.bboxQuaternion = qfinal;
  boxq.bboxTransform = tfinal;
  boxq.cube_length = max_pt.x - min_pt.x;
  boxq.cube_width = max_pt.y - min_pt.y;
  boxq.cube_height = max_pt.z - min_pt.z;

  return boxq;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
