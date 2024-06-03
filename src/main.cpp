#include <iostream>
#include "happly.h"
#include "BoundingBox.hh"
#include "Octree.hh"

int main() {
    // Cargamos el archivo PLY utilizando la API de happly
    happly::PLYData plyIn("mesh.ply", true);

    // Cargamos la malla
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

    // Calculamos la bounding box global
    BoundingBox globalBounds = calculateGlobalBoundingBox(vPos);

    // Creamos un vector de triángulos con sus datos (vértices, normal y proyecciones)
    std::vector<Triangle> triangles;
    triangles.reserve(fInd.size());
    for (const auto& face : fInd) {
        Triangle triangle;
        for (size_t i = 0; i < 3; ++i) {
            Vector3D vertex;
            vertex[0] = vPos[face[i]][0];
            vertex[1] = vPos[face[i]][1];
            vertex[2] = vPos[face[i]][2];
            triangle.vertices[i] = vertex;
        }
        // Calculamos la normal del triángulo
        triangle.normal = glm::normalize(glm::cross(triangle.vertices[1] - triangle.vertices[0], triangle.vertices[2] - triangle.vertices[0]));

        // Calculamos las proyecciones necesarias para SAT
        // (Se deben calcular las 13 proyecciones aquí)

        triangles.push_back(triangle);
    }

    // Profundidad 4 (provisional)
    Octree octree(globalBounds, 6);
    std::cout << "He llegado aqui" << std::endl;
    octree.build(triangles);

    std::cout << "Finished octree generation" << std::endl;

    return 0;
}
