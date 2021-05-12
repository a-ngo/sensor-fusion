// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <vector>

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // generate point clouds
  double groundSlope = 0.0;
  // Lidar lidar(cars, groundSlope);
  Lidar *lidar = new Lidar(cars, groundSlope);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();
  // renderRays(viewer, lidar->position,pointCloud);
  // renderPointCloud(viewer, pointCloud, "pointCloud", Color(1,1,1));

  // process point clouds
  // segmentate plane
  ProcessPointClouds<pcl::PointXYZ> processPointClouds;

  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmentedClouds = processPointClouds.SegmentPlane(pointCloud, 100, 0.2f);
  // renderPointCloud(viewer, segmentedClouds.first, "obstacleCloud",
  //                  Color(1, 0, 0));
  renderPointCloud(viewer, segmentedClouds.second, "planeCloud",
                   Color(1, 1, 1));

  // cluster objects
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters =
      processPointClouds.Clustering(segmentedClouds.first, 2.0, 3, 30);

  int cluster_id{0};
  std::vector<Color> colors{Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_clusters) {
    std::cout << "cluster size ";
    processPointClouds.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(cluster_id),
                     colors[cluster_id]);
    Box box = processPointClouds.BoundingBox(cluster);
    renderBox(viewer, box, cluster_id);
    ++cluster_id;
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  simpleHighway(viewer);

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}