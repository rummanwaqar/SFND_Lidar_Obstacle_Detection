#pragma once

#include <cmath>
#include <cstdint>
#include <pcl/common/common.h>
#include <vector>

// Structure to represent node of kd tree
template <typename PointT> struct Node {
  PointT point;
  int id;
  Node *left;
  Node *right;

  Node(const PointT &p, int setId)
      : point(p), id(setId), left(nullptr), right(nullptr) {}
};

template <typename PointT> struct KdTree {
  Node<PointT> *root;

  KdTree() : root(nullptr) {}

  void insert(const PointT &point, int id) {
    KdTree::insertHelper(root, 0, point, id);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(const PointT &target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }

private:
  static void insertHelper(Node<PointT> *&node, uint8_t depth,
                           const PointT &point, int id) {
    if (!node) { // tree is empty
      node = new Node<PointT>(point, id);
    } else {
      // point index for comparison based on depth
      uint8_t point_index = depth % 3;
      bool comparison;
      switch (point_index) {
      case 0:
        comparison = point.x < node->point.x;
        break;
      case 1:
        comparison = point.y < node->point.y;
        break;
      case 2:
        comparison = point.z < node->point.z;
        break;
      }
      if (comparison) {
        KdTree::insertHelper(node->left, depth + 1, point, id);
      } else {
        KdTree::insertHelper(node->right, depth + 1, point, id);
      }
    }
  }

  static void searchHelper(const PointT &target, Node<PointT> *&node,
                           uint8_t depth, float distanceTol,
                           std::vector<int> &ids) {
    if (node) {
      // check if node is in target box
      if (node->point.x >= (target.x - distanceTol) &&
          node->point.x < (target.x + distanceTol) &&
          node->point.y >= (target.y - distanceTol) &&
          node->point.y < (target.y + distanceTol) &&
          node->point.z >= (target.z - distanceTol) &&
          node->point.z < (target.z + distanceTol)) {
        double distance = std::sqrt(::pow(node->point.x - target.x, 2) +
                                    ::pow(node->point.y - target.y, 2) +
                                    ::pow(node->point.z - target.z, 2));
        if (distance <= distanceTol)
          ids.push_back(node->id);
      }

      double target_val, point_val;
      uint8_t comp_index = depth % 3;
      switch (comp_index) {
      case 0:
        target_val = target.x;
        point_val = node->point.x;
        break;
      case 1:
        target_val = target.y;
        point_val = node->point.y;
        break;
      case 2:
        target_val = target.z;
        point_val = node->point.z;
        break;
      }
      if (target_val - distanceTol < point_val) {
        searchHelper(target, node->left, depth + 1, distanceTol, ids);
      }
      if (target_val + distanceTol > point_val) {
        searchHelper(target, node->right, depth + 1, distanceTol, ids);
      }
    }
  }
};

namespace impl {
template <typename PointT>
void clusterHelper(int i, typename pcl::PointCloud<PointT>::Ptr cloud,
                   pcl::PointIndices &cluster, std::vector<bool> &processed,
                   KdTree<PointT> *tree, float distanceTol) {
  processed[i] = true;
  cluster.indices.push_back(i);
  std::vector<int> nearby_points = tree->search(cloud->points[i], distanceTol);
  for (int id : nearby_points) {
    if (!processed[id]) {
      clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
    }
  }
}
} // namespace impl

template <typename PointT>
std::vector<pcl::PointIndices>
euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud,
                 KdTree<PointT> *tree, float distanceTol, float minSize) {
  std::vector<pcl::PointIndices> clusters;
  std::vector<bool> processed(cloud->points.size(), false);

  for (int i = 0; i < cloud->points.size(); ++i) {
    if (processed[i])
      continue;
    pcl::PointIndices cluster;
    impl::clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
    if (cluster.indices.size() > minSize)
      clusters.push_back(cluster);
  }
  return clusters;
}
