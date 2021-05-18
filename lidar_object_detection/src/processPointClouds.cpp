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
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO(a-ngo): deeper understand this definition
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
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

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
    int min_size, int max_size) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (const auto &idx : it->indices) {
      cluster->push_back((*cloud)[idx]);
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
