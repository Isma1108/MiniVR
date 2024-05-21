#ifndef OCTREE_NODE_HH
#define OCTREE_NODE_HH

#include "BoundingBox.hh"
#include <vector>
#include <array>
#include <memory>

enum class NodeColor { WHITE, BLACK };

struct OctreeNode {
    BoundingBox bounds;
    std::vector<std::vector<std::array<double, 3>>> triangles; // Llista de tringles
    std::unique_ptr<OctreeNode> children[8];
    NodeColor color;

    bool isLeaf() const {
        return children[0] == nullptr;
    }
};

#endif // OCTREE_NODE_HH

