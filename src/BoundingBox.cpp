#include "BoundingBox.hh"
#include <algorithm>
#include <limits>
#include <iostream>


// Projection of points onto an axis (array version)
void project(const std::array<Vector3D, 3>& points, const Vector3D& axis, Real& min, Real& max) {
    min = std::numeric_limits<Real>::max();
    max = std::numeric_limits<Real>::lowest();
    for (const auto& p : points) {
        Real val = glm::dot(axis, p);
        if (val < min) min = val;
        if (val > max) max = val;
    }
}

// Vector version
void project(const std::vector<Vector3D>& points, const Vector3D& axis, Real& min, Real& max) {
    min = std::numeric_limits<Real>::max();
    max = std::numeric_limits<Real>::lowest();
    for (const auto& p : points) {
        Real val = glm::dot(axis, p);
        if (val < min) min = val;
        if (val > max) max = val;
    }
}

// Function to check if exists intersection between a triangle and a bounding box using the Separating Axis Theorem
// It uses a Fast 3D Triangle-Box Overlap Testing approach based on 
        //Tomas Akenine-Moller. (2001, March). Fast 3D Triangle-Box Overlap Testing.
bool BoundingBox::intersects(const Triangle& triangle) const {
    std::vector<Vector3D> boxVertices(8);
    
    // Generate the 8 vertices of the bounding box
    for (int i = 0; i < 8; ++i) {
        boxVertices[i] = min;
        if (i & 1) boxVertices[i].x = max.x;
        if (i & 2) boxVertices[i].y = max.y;
        if (i & 4) boxVertices[i].z = max.z;
    }

    // Axes of the box (normals of the x, y, z axes)
    Vector3D boxNormals[3] = {
        Vector3D(1, 0, 0),
        Vector3D(0, 1, 0),
        Vector3D(0, 0, 1)
    };

    // Test the box axes
    for (int i = 0; i < 3; ++i) {
        Real triangleMin, triangleMax;
        project(triangle.vertices, boxNormals[i], triangleMin, triangleMax);
        if (triangleMax < min[i] || triangleMin > max[i]) {
            //std::cout << "falseee" << std::endl;
            return false; // No intersection possible
        }
    }

    // Test the triangle's axis
    Vector3D triangleNorm = glm::cross(triangle.vertices[1] - triangle.vertices[0], triangle.vertices[2] - triangle.vertices[0]);
    Real triangleOffset = glm::dot(triangleNorm, triangle.vertices[0]);
    Real boxMin, boxMax;
    project(boxVertices, triangleNorm, boxMin, boxMax);
    if (boxMax < triangleOffset || boxMin > triangleOffset) {
        //std::cout << "falseee" << std::endl;
        return false; // No intersection possible
    }

    // Test the nine cross-products of the edges
    Vector3D triangleEdges[3] = {
        triangle.vertices[0] - triangle.vertices[1],
        triangle.vertices[1] - triangle.vertices[2],
        triangle.vertices[2] - triangle.vertices[0]
    };
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector3D axis = glm::cross(triangleEdges[i], boxNormals[j]);
            project(boxVertices, axis, boxMin, boxMax);
            Real triangleMin, triangleMax;
            project(triangle.vertices, axis, triangleMin, triangleMax);
            if (boxMax < triangleMin || boxMin > triangleMax) {
                //std::cout << "falseee" << std::endl;
                return false; // No intersection possible
            }
        }
    }
    //std::cout << "trueee" << std::endl;
    // No separating axis found, therefore there is an intersection
    return true;
}

BoundingBox calculateGlobalBoundingBox(const std::vector<std::array<double, 3>>& vertexPositions) {
    BoundingBox bbox;
    bbox.min = Vector3D(std::numeric_limits<Real>::max(), std::numeric_limits<Real>::max(), std::numeric_limits<Real>::max());
    bbox.max = Vector3D(std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::lowest());

    for (const auto& pos : vertexPositions) {
        for (int i = 0; i < 3; ++i) {
            bbox.min[i] = std::min(bbox.min[i], static_cast<Real>(pos[i]));
            bbox.max[i] = std::max(bbox.max[i], static_cast<Real>(pos[i]));
        }
    }

    Real sideLength = std::max({bbox.max[0] - bbox.min[0], bbox.max[1] - bbox.min[1], bbox.max[2] - bbox.min[2]});
    Vector3D center = (bbox.min + bbox.max) / static_cast<Real>(2.0);

    bbox.min = center - Vector3D(sideLength / static_cast<Real>(2.0));
    bbox.max = center + Vector3D(sideLength / static_cast<Real>(2.0));

    return bbox;
}


