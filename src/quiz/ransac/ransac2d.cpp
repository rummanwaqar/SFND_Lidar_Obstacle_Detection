/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
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

struct LineModel {
  double a{0};
  double b{0};
  double c{0};

  LineModel() = default;

  LineModel(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2) {
    this->a = point1.y - point2.y;
    this->b = point2.x - point1.x;
    this->c = point1.x * point2.y - point2.x * point1.y;
  }
};

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult{};
  srand(time(NULL));

  // For max iterations
  while (maxIterations--) {
    // Randomly sample subset and fit line
    auto point1 = cloud->points[rand() % cloud->points.size()];
    auto point2 = cloud->points[rand() % cloud->points.size()];
    LineModel line{point1, point2};

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    std::unordered_set<int> inliers;
    double dist_denom = std::sqrt(line.a * line.a + line.b * line.b);
    for (int i = 0; i < cloud->points.size(); ++i) {
      auto point = cloud->points[i];
      double distance =
          std::fabs(line.a * point.x + line.b * point.y + line.c) / dist_denom;
      if (distance <= distanceTol) {
        inliers.insert(i);
      }
    }
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  return inliersResult;
}

struct PlaneModel {
  double a{0};
  double b{0};
  double c{0};
  double d{0};

  double denom{1};

  PlaneModel() = default;

  PlaneModel(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2,
             const pcl::PointXYZ &point3) {
    const pcl::PointXYZ v1 = {point2.x - point1.x, point2.y - point1.y,
                              point2.z - point1.z};
    const pcl::PointXYZ v2 = {point3.x - point1.x, point3.y - point1.y,
                              point3.z - point1.z};

    double i = v1.y * v2.z - v1.z * v2.y;
    double j = -(v1.x * v2.z - v1.z * v2.x);
    double k = (v1.x * v2.y - v1.y * v2.x);

    this->a = i;
    this->b = j;
    this->c = k;
    this->d = -(i * point1.x + j * point1.y + k * point1.z);

    this->denom = ::sqrt(a * a + b * b + c * c);
  }

  double distanceFromPoint(const pcl::PointXYZ &point) {
    return ::fabs(a * point.x + b * point.y + c * point.z + d) / this->denom;
  }
};

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult{};
  srand(time(NULL));

  while (maxIterations--) {
    std::unordered_set<int> inliers{};

    while (inliers.size() < 3) {
      inliers.insert(rand() % cloud->points.size());
    }
    // Randomly sample subset and fit line
    auto it = inliers.begin();
    auto point1 = cloud->points[*it];
    it++;
    auto point2 = cloud->points[*it];
    it++;
    auto point3 = cloud->points[*it];
    PlaneModel plane{point1, point2, point3};

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    for (int i = 0; i < cloud->points.size(); ++i) {
      if (inliers.count(i) > 0)
        continue;

      auto point = cloud->points[i];
      if (plane.distanceFromPoint(point) <= distanceTol) {
        inliers.insert(i);
      }
    }
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  return inliersResult;
}

int main() {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

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
