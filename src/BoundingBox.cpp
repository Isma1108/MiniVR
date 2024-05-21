#include "BoundingBox.hh"
#include <algorithm>

bool BoundingBox::intersects(const std::vector<std::array<double, 3>>& triangle) const {
    // AQUI HE D'IMPLEMENTAR QUALSEVOL ALGORISME DE TEST INTERSECCIO BOX TRIANGLE,
    // IDEALMENT EL Fast Separating Axis Theorem
    return true;
}

BoundingBox calculateGlobalBoundingBox(const std::vector<std::array<double, 3>>& vertexPositions) {
    BoundingBox bbox;
    bbox.min = vertexPositions[0];
    bbox.max = vertexPositions[0];

    for (const auto& pos : vertexPositions) {
        for (int i = 0; i < 3; ++i) {
            bbox.min[i] = std::min(bbox.min[i], pos[i]);
            bbox.max[i] = std::max(bbox.max[i], pos[i]);
        }
    }
    return bbox;
}

