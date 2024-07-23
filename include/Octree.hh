#ifndef OCTREE_HH
#define OCTREE_HH

#include "OctreeNode.hh"
#include "Types.hh"
#include "BoundingBox.hh"
#include <vector>
#include <memory>

class Octree {
public:
    Octree(const BoundingBox& bounds, int maxDepth);

    void build(const std::vector<Triangle>& triangles);

    void setUseGPU(bool useGPU) {
        this->useGPU = useGPU;
    }

    OctreeNode* getRoot() const {
        return root.get();
    }

    std::vector<OctreeNode*> collectGreyLeafNodes() const;

private:
    std::unique_ptr<OctreeNode> root;
    int maxDepth;
    const std::vector<Triangle>* allTriangles;
    bool useGPU;

    void buildRecursive(OctreeNode* node, int depth);
    void buildRecursiveGPU(OctreeNode* node, int depth);
    void buildRecursiveCPU(OctreeNode* node, int depth);

    void subdivide(OctreeNode* node);
    bool isInsideModel(const BoundingBox& box) const;

    void collectGreyLeafNodesRecursive(OctreeNode* node, std::vector<OctreeNode*>& greyNodes) const;
};

#endif // OCTREE_HH
