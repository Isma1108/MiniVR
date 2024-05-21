#ifndef OCTREE_HH
#define OCTREE_HH

#include "OctreeNode.hh"

class Octree {
public:
    Octree(const BoundingBox& bounds, int maxDepth);

    void build(const std::vector<std::vector<std::array<double, 3>>>& triangles);

private:
    std::unique_ptr<OctreeNode> root;
    int maxDepth;
    const std::vector<std::vector<std::array<double, 3>>>* allTriangles;

    void buildRecursive(OctreeNode* node, int depth);
    void subdivide(OctreeNode* node);
    bool isInsideModel(const BoundingBox& box) const;
};

#endif // OCTREE_HH

