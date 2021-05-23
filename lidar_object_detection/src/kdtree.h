#ifndef KDRTREE_H_
#define KDRTREE_H_

#include "render/render.h"
#include <math.h>
#include <sys/types.h>
#include <vector>

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  ~KdTree() { delete root; }

  void insert_helper(Node **node, uint depth, std::vector<float> point,
                     int id) {
    // TODO(a-ngo): deeply understand these pointers
    // dereference to check its value
    if (*node == NULL) {
      *node = new Node(point, id);
    } else {
      // current dimension
      uint dim = depth % 2;

      if (point[dim] < (*node)->point[dim]) {
        insert_helper(&(*node)->left, depth + 1, point, id);
      } else {
        insert_helper(&(*node)->right, depth + 1, point, id);
      }
    }
  }

  void insert(std::vector<float> point, int id) {
    insert_helper(&root, 0, point, id);
  }

  void search_helper(Node *node, uint depth, std::vector<int> &ids,
                     std::vector<float> target, float distance_tol) {
    if (node != NULL) {
      if (fabs(node->point[0] - target[0]) <= distance_tol &&
          fabs(node->point[1] - target[1]) <= distance_tol) {
        // calculate distance of target and node -> if smaller then add node id

        float x = node->point[0] - target[0];
        float y = node->point[1] - target[1];
        float distance = sqrt(pow(x, 2) + pow(y, 2));

        if (distance <= distance_tol) {
          ids.push_back(node->id);
        }
      }

      uint dim = depth % 2;
      if ((target[dim] - distance_tol) < node->point[dim])
        search_helper(node->left, depth + 1, ids, target, distance_tol);
      if ((target[dim] + distance_tol) > node->point[dim])
        search_helper(node->right, depth + 1, ids, target, distance_tol);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distance_tol) {
    std::vector<int> ids;
    // start with root and traverse through the tree
    search_helper(root, 0, ids, target, distance_tol);

    return ids;
  }
};
#endif /* KDTREE_H_ */
