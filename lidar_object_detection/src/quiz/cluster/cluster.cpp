#include "../../render/box.h"
#include "../../render/render.h"
#include "kdtree.h"
#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);

  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0,
                  0.1, 0.1, 0.1, "window");
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
CreateData(std::vector<std::vector<float>> points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int i = 0; i < points.size(); i++) {
    pcl::PointXYZ point;
    point.x = points[i][0];
    point.y = points[i][1];
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

void render2DTree(Node *node, pcl::visualization::PCLVisualizer::Ptr &viewer,
                  Box window, int &iteration, uint depth = 0) {
  if (node != NULL) {
    Box upperWindow = window;
    Box lowerWindow = window;
    // split on x axis
    if (depth % 2 == 0) {
      viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),
                      pcl::PointXYZ(node->point[0], window.y_max, 0), 0, 0, 1,
                      "line" + std::to_string(iteration));
      lowerWindow.x_max = node->point[0];
      upperWindow.x_min = node->point[0];
    } else {
      // split on y axis
      viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),
                      pcl::PointXYZ(window.x_max, node->point[1], 0), 1, 0, 0,
                      "line" + std::to_string(iteration));
      lowerWindow.y_max = node->point[1];
      upperWindow.y_min = node->point[1];
    }
    iteration++;

    render2DTree(node->left, viewer, lowerWindow, iteration, depth + 1);
    render2DTree(node->right, viewer, upperWindow, iteration, depth + 1);
  }
}

void proximity(const std::vector<float> point, std::vector<int> &cluster,
               KdTree *tree, float distance_tol, unsigned int point_id,
               std::unordered_set<int> &processed_points) {
  if (!processed_points.count(point_id)) {
    processed_points.insert(point_id);
    cluster.push_back(point_id);
    std::vector<int> nearby_point_ids = tree->search(point, distance_tol);
    for (int id : nearby_point_ids) {
      proximity(point, cluster, tree, distance_tol, id, processed_points);
    }
  }
}

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree,
                 float distance_tol) {
  std::vector<std::vector<int>> clusters;
  std::unordered_set<int> processed_points;
  unsigned int point_id{0};
  for (const std::vector<float> point : points) {
    if (!processed_points.count(point_id)) {
      std::vector<int> cluster;
      // find and add all nearby points to cluster
      proximity(point, cluster, tree, distance_tol, point_id, processed_points);
      clusters.push_back(cluster);
    }
    ++point_id;
  }
  return clusters;
}

int main() {
  // Create viewer
  Box window;
  window.x_min = -10;
  window.x_max = 10;
  window.y_min = -10;
  window.y_max = 10;
  window.z_min = 0;
  window.z_max = 0;
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

  // Create data
  std::vector<std::vector<float>> points = {
      {-6.2, 7},   {-6.3, 8.4},  {-5.2, 7.1}, {-5.7, 6.3},
      {7.2, 6.1},  {8.0, 5.3},   {7.2, 7.1},  {0.2, -7.1},
      {1.7, -6.9}, {-1.2, -7.2}, {2.2, -8.9}};
  // std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4},
  // {-5.2,7.1}, {-5.7,6.3} };
  //   std::vector<std::vector<float>> points = {{-6.2, 7}};
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
  std::cout << "There are " << points.size() << " points." << std::endl;

  KdTree *tree = new KdTree;

  for (int i = 0; i < points.size(); i++)
    tree->insert(points[i], i);

  int it = 0;
  render2DTree(tree->root, viewer, window, it);

  std::cout << "Test Search" << std::endl;
  // visualize target zone
  std::vector<float> target{-6, 3};
  float distance_tol = 5.0;
  viewer->addCube(target[0] - distance_tol, target[0] + distance_tol,
                  target[1] - distance_tol, target[1] + distance_tol, 0, 0, 0.0,
                  0.8, 0.0, "target");
  pcl::ModelCoefficients circle_coeff;
  circle_coeff.values.resize(3);
  circle_coeff.values[0] = target[0];
  circle_coeff.values[1] = target[1];
  circle_coeff.values[2] = distance_tol;
  viewer->addCircle(circle_coeff, "distance_circle");

  std::vector<int> nearby = tree->search(target, distance_tol);
  for (int index : nearby)
    std::cout << index << ",";
  std::cout << std::endl;

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering found " << clusters.size() << " and took "
            << elapsedTime.count() << " milliseconds" << std::endl;

  // Render clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for (std::vector<int> cluster : clusters) {
    std::cout << "Cluster " << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (int index : cluster) {
      std::cout << "index = " << index << std::endl;
      clusterCloud->points.push_back(
          pcl::PointXYZ(points[index][0], points[index][1], 0));
    }
    renderPointCloud(viewer, clusterCloud,
                     "cluster" + std::to_string(clusterId),
                     colors[clusterId % 3]);
    ++clusterId;
  }
  if (clusters.size() == 0)
    renderPointCloud(viewer, cloud, "data");

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
