#include "Octree.hh"

Octree::Octree(const BoundingBox& bounds, int maxDepth)
    : root(std::make_unique<OctreeNode>(OctreeNode{bounds, NodeColor::GREY})), maxDepth(maxDepth), allTriangles(nullptr) {}

void Octree::build(const std::vector<Triangle>& triangles) {
    allTriangles = &triangles;
    root->triangleIndices.reserve(triangles.size());
    for (size_t i = 0; i < triangles.size(); ++i) {
        root->triangleIndices.push_back(i);
    }
    buildRecursive(root.get(), 0);
}

void Octree::buildRecursive(OctreeNode* node, int depth) {
    if (depth >= maxDepth || node->triangleIndices.empty()) {
        return;
    }

    subdivide(node);

    //std::vector<size_t> childTriangleIndices[8];
    for (size_t idx : node->triangleIndices) {
        const auto& triangle = (*allTriangles)[idx];
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]->bounds.intersects(triangle)) {
                node->children[i]->triangleIndices.push_back(idx);
                //childTriangleIndices[i].push_back(idx);
            }
        }
    }

    for (int i = 0; i < 8; ++i) {
        //node->children[i]->triangleIndices = std::move(childTriangleIndices[i]);

        if (node->children[i]->triangleIndices.empty()) {
            if (isInsideModel(node->children[i]->bounds)) {
                node->children[i]->color = NodeColor::BLACK;
            } else {
                node->children[i]->color = NodeColor::WHITE;
            }
        } else {
            buildRecursive(node->children[i].get(), depth + 1);
        }
    }

    node->triangleIndices.clear();
}

void Octree::subdivide(OctreeNode* node) {
    auto& bounds = node->bounds;
    Vector3D mid = (bounds.min + bounds.max) / static_cast<Real>(2.0);

    for (int i = 0; i < 8; ++i) {
        Vector3D newMin = bounds.min;
        Vector3D newMax = mid;

        if (i & 1) newMin.x = mid.x; else newMax.x = bounds.max.x;
        if (i & 2) newMin.y = mid.y; else newMax.y = bounds.max.y;
        if (i & 4) newMin.z = mid.z; else newMax.z = bounds.max.z;

        node->children[i] = std::make_unique<OctreeNode>(newMin, newMax, NodeColor::GREY);
    }
}

bool Octree::isInsideModel(const BoundingBox& box) const {
    // Aquí falta implementar el test para saber si la caja delimitadora está dentro o fuera del modelo
    return true;
}


