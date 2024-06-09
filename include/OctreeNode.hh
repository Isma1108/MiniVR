#ifndef OCTREE_NODE_HH
#define OCTREE_NODE_HH

#include "BoundingBox.hh"
#include "Types.hh"
#include <memory>
#include <vector>

// WHITE -> Outside the model
// BLACK -> Inside the model
// GREY  -> Intersecting the model
enum class NodeColor { WHITE, BLACK, GREY };

struct OctreeNode {
    BoundingBox bounds;
    std::vector<size_t> triangleIndices;
    std::unique_ptr<OctreeNode> children[8];
    NodeColor color;
    int depth;

    OctreeNode(const BoundingBox& b, NodeColor col)
        : bounds(b), color(col) {}

    OctreeNode(const Vector3D& min, const Vector3D& max, NodeColor col)
        : bounds({min, max}), color(col) {}

    bool isLeaf() const {
        return children[0] == nullptr;
    }
};

#endif // OCTREE_NODE_HH
