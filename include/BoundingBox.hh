#ifndef BOUNDING_BOX_HH
#define BOUNDING_BOX_HH

#include <array>
#include <vector>

struct BoundingBox {
    std::array<double, 3> min;
    std::array<double, 3> max;

    bool intersects(const std::vector<std::array<double, 3>>& triangle) const;
};

BoundingBox calculateGlobalBoundingBox(const std::vector<std::array<double, 3>>& vertexPositions);

#endif // BOUNDING_BOX_HH

