#include "Octree.hh"
#include <iostream>

Octree::Octree(const BoundingBox& bounds, int maxDepth)
    : root(std::make_unique<OctreeNode>(OctreeNode{bounds, NodeColor::GREY})), maxDepth(maxDepth), allTriangles(nullptr) {}

void Octree::build(const std::vector<Triangle>& triangles) {
    allTriangles = &triangles;
    root->triangleIndices.reserve(triangles.size());
    for (size_t i = 0; i < triangles.size(); ++i) {
        root->triangleIndices.push_back(i);
    }
    root->color = NodeColor::GREY;
    root->depth = 0;
    buildRecursive(root.get(), 0);
}

void Octree::buildRecursive(OctreeNode* node, int depth) {
    if (depth >= maxDepth ){//|| node->triangleIndices.empty()) {
        return;
    }

    subdivide(node);
    for (int i = 0; i < 8; ++i) {
        node->depth = depth;    }

    //std::vector<size_t> childTriangleIndices[8];
    for (size_t idx : node->triangleIndices) {
        const auto& triangle = (*allTriangles)[idx];
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]->bounds.intersects(triangle)) {
                node->children[i]->triangleIndices.push_back(idx);
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

/*
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
*/


void Octree::subdivide(OctreeNode* node) {
    auto& bounds = node->bounds;
    Vector3D mid = (bounds.min + bounds.max) / static_cast<Real>(2.0);

    // Create all 8 children with correct bounds
    node->children[0] = std::make_unique<OctreeNode>(bounds.min, mid, NodeColor::GREY); // 000
    node->children[1] = std::make_unique<OctreeNode>(
        Vector3D(mid.x, bounds.min.y, bounds.min.z), 
        Vector3D(bounds.max.x, mid.y, mid.z), 
        NodeColor::GREY); // 001
    node->children[2] = std::make_unique<OctreeNode>(
        Vector3D(bounds.min.x, mid.y, bounds.min.z), 
        Vector3D(mid.x, bounds.max.y, mid.z), 
        NodeColor::GREY); // 010
    node->children[3] = std::make_unique<OctreeNode>(
        Vector3D(mid.x, mid.y, bounds.min.z), 
        Vector3D(bounds.max.x, bounds.max.y, mid.z), 
        NodeColor::GREY); // 011
    node->children[4] = std::make_unique<OctreeNode>(
        Vector3D(bounds.min.x, bounds.min.y, mid.z), 
        Vector3D(mid.x, mid.y, bounds.max.z), 
        NodeColor::GREY); // 100
    node->children[5] = std::make_unique<OctreeNode>(
        Vector3D(mid.x, bounds.min.y, mid.z), 
        Vector3D(bounds.max.x, mid.y, bounds.max.z), 
        NodeColor::GREY); // 101
    node->children[6] = std::make_unique<OctreeNode>(
        Vector3D(bounds.min.x, mid.y, mid.z), 
        Vector3D(mid.x, bounds.max.y, bounds.max.z), 
        NodeColor::GREY); // 110
    node->children[7] = std::make_unique<OctreeNode>(
        Vector3D(mid.x, mid.y, mid.z), 
        Vector3D(bounds.max.x, bounds.max.y, bounds.max.z), 
        NodeColor::GREY); // 111
}

bool Octree::isInsideModel(const BoundingBox& box) const {
    // Aquí falta implementar el test para saber si la caja delimitadora está dentro o fuera del modelo
    return true;
}

std::vector<OctreeNode*> Octree::collectGreyLeafNodes() const {
    std::vector<OctreeNode*> greyNodes;
    collectGreyLeafNodesRecursive(root.get(), greyNodes);
    return greyNodes;
}

void Octree::collectGreyLeafNodesRecursive(OctreeNode* node, std::vector<OctreeNode*>& greyNodes) const {
    if (node->isLeaf() && node->color == NodeColor::GREY) { 
        greyNodes.push_back(node);
    } else {
        for (const auto& child : node->children) {
            if (child) {
                collectGreyLeafNodesRecursive(child.get(), greyNodes);
            }
        }
    }
}


