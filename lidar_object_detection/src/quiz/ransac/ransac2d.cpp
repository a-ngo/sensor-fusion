#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <cstdint>
#include <math.h>
#include <unordered_set>
#include <vector>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * ((static_cast<double>(rand()) / (RAND_MAX)) - 0.5);
    double ry = 2 * ((static_cast<double>(rand()) / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * ((static_cast<double>(rand()) / (RAND_MAX)) - 0.5);
    double ry = 2 * ((static_cast<double>(rand()) / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

unsigned int random_number_in_range(unsigned int range) {
  return rand() % range;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int max_iterations, float distance_tol) {
  std::unordered_set<int> inliers_result;
  srand(time(NULL));
  std::cout << "This cloud has " << cloud->points.size() << " points."
            << std::endl;

  auto max_number{cloud->points.size()};

  for (int64_t iteration{0}; iteration <= max_iterations; ++iteration) {
    std::unordered_set<int> inliers;

    // Randomly sample subset and fit line
    while (inliers.size() < 3) {
      inliers.insert(random_number_in_range(max_number));
    }

    auto itr = inliers.begin();
    pcl::PointXYZ point_one = cloud->points[*itr];
    ++itr;
    pcl::PointXYZ point_two = cloud->points[*itr];
    ++itr;
    pcl::PointXYZ point_three = cloud->points[*itr];

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

      pcl::PointXYZ point = cloud->points[index];
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

  std::cout << "inlier counter = " << inliers_result.size() << std::endl;

  return inliers_result;
}

int main() {
  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5f);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
