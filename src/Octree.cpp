#include "Octree.hh"

Octree::Octree(const BoundingBox& bounds, int maxDepth)
    : root(std::make_unique<OctreeNode>(OctreeNode{bounds, {}, NodeColor::WHITE})), maxDepth(maxDepth), allTriangles(nullptr) {}

void Octree::build(const std::vector<std::vector<std::array<double, 3>>>& triangles) {
    allTriangles = &triangles;
    root->triangles = triangles;
    buildRecursive(root.get(), 0);
}

void Octree::buildRecursive(OctreeNode* node, int depth) {
    if (depth >= maxDepth || node->triangles.empty()) {
        return;
    }

    subdivide(node);

    std::vector<std::vector<std::array<double, 3>>> childTriangles[8];
    for (const auto& triangle : node->triangles) {
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]->bounds.intersects(triangle)) {
                childTriangles[i].push_back(triangle);
            }
        }
    }

    for (int i = 0; i < 8; ++i) {
        node->children[i]->triangles = childTriangles[i];

        if (node->children[i]->triangles.empty()) {
            if (isInsideModel(node->children[i]->bounds)) {
                node->children[i]->color = NodeColor::BLACK;
            } else {
                node->children[i]->color = NodeColor::WHITE;
            }
        }
        else buildRecursive(node->children[i].get(), depth + 1);
    }

    node->triangles.clear();
}

void Octree::subdivide(OctreeNode* node) {
    auto& bounds = node->bounds;
    std::array<double, 3> mid = {
        (bounds.min[0] + bounds.max[0]) / 2,
        (bounds.min[1] + bounds.max[1]) / 2,
        (bounds.min[2] + bounds.max[2]) / 2
    };

    for (int i = 0; i < 8; ++i) {
        std::array<double, 3> newMin = bounds.min;
        std::array<double, 3> newMax = mid;

        if (i & 1) newMin[0] = mid[0]; else newMax[0] = bounds.max[0];
        if (i & 2) newMin[1] = mid[1]; else newMax[1] = bounds.max[1];
        if (i & 4) newMin[2] = mid[2]; else newMax[2] = bounds.max[2];

        node->children[i] = std::make_unique<OctreeNode>(OctreeNode{{newMin, newMax}, {}, NodeColor::WHITE});
    }
}

bool Octree::isInsideModel(const BoundingBox& box) const {
    // AQUI FALTA IMPLEMENTAR EL TEST PER SABER SI LA BOUNDING BOX ES DINS O FORA DEL MODEL
    return true;
}

