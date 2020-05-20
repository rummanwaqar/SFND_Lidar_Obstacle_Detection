/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(nullptr), right(nullptr) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(nullptr) {}

  void insert(const std::vector<float> &point, int id) {
    KdTree::insertHelper(root, 0, point, id);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }

private:
  static void insertHelper(Node *&node, uint depth,
                           const std::vector<float> &point, int id) {
    if (!node) { // tree is empty
      node = new Node(point, id);
    } else {
      // point index for comparison based on depth
      uint point_index = depth % 2;
      if (point[point_index] < node->point[point_index]) {
        KdTree::insertHelper(node->left, depth + 1, point, id);
      } else {
        KdTree::insertHelper(node->right, depth + 1, point, id);
      }
    }
  }

  static void searchHelper(const std::vector<float> &target, Node *&node,
                           uint depth, float distanceTol,
                           std::vector<int> &ids) {
    if (node) {
      // check if node is in target box
      if (node->point[0] >= (target[0] - distanceTol) &&
          node->point[0] < (target[0] + distanceTol) &&
          node->point[1] >= (target[1] - distanceTol) &&
          node->point[1] < (target[1] + distanceTol)) {
        double distance = std::sqrt(::pow(node->point[0] - target[0], 2) +
                                    ::pow(node->point[1] - target[1], 2));
        if (distance <= distanceTol)
          ids.push_back(node->id);
      }

      uint comp_index = depth % 2;
      if (target[comp_index] - distanceTol < node->point[comp_index]) {
        searchHelper(target, node->left, depth + 1, distanceTol, ids);
      }
      if (target[comp_index] + distanceTol > node->point[comp_index]) {
        searchHelper(target, node->right, depth + 1, distanceTol, ids);
      }
    }
  }
};
