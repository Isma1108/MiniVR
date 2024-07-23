#ifndef BOUNDING_BOX_HH
#define BOUNDING_BOX_HH

#include <array>
#include <vector>
#include <glm/glm.hpp>
#include "Types.hh"

struct BoundingBox {
    Vector3D min;
    Vector3D max;

    bool intersects(const Triangle& triangle) const;
};

BoundingBox calculateGlobalBoundingBox(const std::vector<std::array<double, 3>>& vertexPositions);

#endif // BOUNDING_BOX_HH
