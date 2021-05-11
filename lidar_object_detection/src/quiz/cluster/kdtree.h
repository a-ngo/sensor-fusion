/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
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
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    insert_helper(&root, 0, point, id);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    return ids;
  }
};
