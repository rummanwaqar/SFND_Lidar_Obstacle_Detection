#pragma once

#include <pcl/common/common.h>
#include <unordered_set>

template <typename PointT> struct PlaneModel {
  double a{0};
  double b{0};
  double c{0};
  double d{0};

  double denom{1};

  PlaneModel() = default;

  PlaneModel(const PointT &point1, const PointT &point2, const PointT &point3) {
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

  double distanceFromPoint(const PointT &point) {
    return ::fabs(a * point.x + b * point.y + c * point.z + d) / this->denom;
  }
};

template <typename PointT>
void RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                 pcl::PointIndices::Ptr inliersResult, int maxIterations,
                 float distanceTol) {
  std::unordered_set<int> inliersTemp;
  srand(time(nullptr));

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
    PlaneModel<PointT> plane{point1, point2, point3};

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
    if (inliers.size() > inliersTemp.size()) {
      inliersTemp = inliers;
    }
  }

  for (auto i : inliersTemp) {
    inliersResult->indices.push_back(i);
  }
}