#include <iostream>
#include "happly.h"
#include "BoundingBox.hh"
#include "Octree.hh"

int main() {
    // We load the PLY file using happly API
    happly::PLYData plyIn("mesh.ply", true);

    // We load the mesh 
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

    // We calculate the global bounding box
    BoundingBox globalBounds = calculateGlobalBoundingBox(vPos);

    std::vector<std::vector<std::array<double, 3>>> triangles;
    for (const auto& face : fInd) {
        triangles.push_back({
            vPos[face[0]],
            vPos[face[1]],
            vPos[face[2]]
        });
    }

    // Depth 5 (provisional)
    Octree octree(globalBounds, 5);
    octree.build(triangles);

    return 0;
}

