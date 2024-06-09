#ifndef OCTREE_HH
#define OCTREE_HH

#include "OctreeNode.hh"
#include "Types.hh"

class Octree {
public:
    Octree(const BoundingBox& bounds, int maxDepth);

    void build(const std::vector<Triangle>& triangles);

    OctreeNode* getRoot() const {
        return root.get();
    }

    std::vector<OctreeNode*> collectGreyLeafNodes() const;

private:
    std::unique_ptr<OctreeNode> root;
    int maxDepth;
    const std::vector<Triangle>* allTriangles;

    void buildRecursive(OctreeNode* node, int depth);
    void subdivide(OctreeNode* node);
    bool isInsideModel(const BoundingBox& box) const;

    void collectGreyLeafNodesRecursive(OctreeNode* node, std::vector<OctreeNode*>& greyNodes) const;
};

#endif // OCTREE_HH
