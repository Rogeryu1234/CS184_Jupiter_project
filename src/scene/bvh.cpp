#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  // Create a new BVH node with the computed bounding box
  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;

  // If the number of primitives in the range is less than or equal to the maximum leaf size,
  // this node becomes a leaf node and we stop recursion
  if (std::distance(start, end) <= max_leaf_size) {
    return node;
  }

  // Otherwise, we need to split the primitives and continue constructing the BVH
  // Find the longest axis of the bounding box
  Vector3D extent = bbox.extent;
  int axis = (extent.x > extent.y) ? ((extent.x > extent.z) ? 0 : 2) : ((extent.y > extent.z) ? 1 : 2);


  // Sort the primitives based on their centroids along the longest axis
  auto mid = start + std::distance(start, end) / 2;
  std::nth_element(start, mid, end, [axis](const Primitive* a, const Primitive* b) {
    return a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis];
  });


  // // Compute the midpoint of the longest axis
  // double split_point = 0.5 * (bbox.min[axis] + bbox.max[axis]);

  // // Partition the primitives into two groups based on the split point
  // auto mid = std::partition(start, end, [axis, split_point](const Primitive* p) {
  //   return p->get_bbox().centroid()[axis] < split_point;
  // });

  // // Check if the split point caused all primitives to end up in one group,
  // // if so, adjust the split point
  // if (mid == start || mid == end) {
  //   mid = start + std::distance(start, end) / 2;
  // }

  // Recursively construct the left and right child nodes
  node->l = construct_bvh(start, mid, max_leaf_size);
  node->r = construct_bvh(mid, end, max_leaf_size);

  return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  double t0;
  double t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false; // No intersection with the bounding box
  }

  // If the current node is a leaf node, check intersection with its primitives
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; ++p) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
        return true; // Intersection found
      }
    }
    return false; // No intersection with any primitive in this leaf node
  }

  // Recursively check for intersection with the left and right children
  total_isects += 1;
  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  double t0;
  double t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false; // No intersection with the bounding box
  }

  bool hit = false;
  //Intersection nearest_intersection = Intersection();
  //double smallest_t = INF_D;

  // If the current node is a leaf node, check intersection with its primitives
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; ++p) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;
    }
    return hit; // Return true if any intersection is found
  }

  total_isects += 1;

  hit = intersect(ray, i, node->l) || hit;
  hit = intersect(ray, i, node->r) || hit;

  return hit;
}

} // namespace SceneObjects
} // namespace CGL
