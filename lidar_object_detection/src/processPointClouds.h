// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include "render/box.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "kdtree.h"

template <typename PointT> class ProcessPointClouds {
public:
  // constructor
  ProcessPointClouds();
  // deconstructor
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr
  FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
              Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  SeparateClouds(pcl::PointIndices::Ptr inliers,
                 typename pcl::PointCloud<PointT>::Ptr cloud);

  std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
                                 int max_iterations, float distance_tol);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
               float distanceThreshold);

  void proximity(const std::vector<float> point, std::vector<int> &cluster,
                 KdTree *tree, float distance_tol, unsigned int point_id,
                 std::unordered_set<int> &processed_points);

  std::vector<std::vector<int>>
  euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree,
                   float distance_tol);

  std::vector<typename pcl::PointCloud<PointT>::Ptr>
  Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
             float clusterTolerance, int minSize, int maxSize);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */
