#include <iostream>
#include <filesystem>
#include "happly.h"
#include "BoundingBox.hh"
#include "Octree.hh"

void showUsage() {
    std::cout << "Usage: <...>/MiniVR [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -satGPU   Use GPU for SAT intersection test in the octree generation.\n";
    std::cout << "  -satCPU   Use CPU for SAT intersection test in the octree generation.\n";
    std::cout << "  You must select one of these options.\n";
}

int main(int argc, char* argv[]) {
    bool useGPU = false;
    bool optionSelected = false;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-satGPU") {
            useGPU = true;
            optionSelected = true;
        } else if (std::string(argv[i]) == "-satCPU") {
            useGPU = false;
            optionSelected = true;
        }
    }

    if (!optionSelected) {
        std::cerr << "Error: You must select one option.\n";
        showUsage();
        return 1;
    }

    int depth = 7;
    std::string selectedFile = "mesh.ply";

    happly::PLYData plyIn(selectedFile, true);
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

    BoundingBox globalBounds = calculateGlobalBoundingBox(vPos);
    Vector3D mid = (globalBounds.min + globalBounds.max) / static_cast<Real>(2.0);

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
        triangle.normal = glm::normalize(glm::cross(triangle.vertices[1] - triangle.vertices[0], triangle.vertices[2] - triangle.vertices[0]));
        triangles.push_back(triangle);
    }

    Octree octree(globalBounds, depth);
    octree.setUseGPU(useGPU);
    octree.build(triangles);

    std::vector<OctreeNode*> greyLeafNodes = octree.collectGreyLeafNodes();

    std::vector<std::array<double, 3>> octreeVertexPositions;
    std::vector<std::vector<size_t>> octreeFaceIndices;

    for (const auto& node : greyLeafNodes) {
        const auto& min = node->bounds.min;
        const auto& max = node->bounds.max;

        size_t startIndex = octreeVertexPositions.size();

        // Add the 8 vertices of the node's bounding box
        octreeVertexPositions.push_back({min.x, min.y, min.z});
        octreeVertexPositions.push_back({max.x, min.y, min.z});
        octreeVertexPositions.push_back({min.x, max.y, min.z});
        octreeVertexPositions.push_back({max.x, max.y, min.z});
        octreeVertexPositions.push_back({min.x, min.y, max.z});
        octreeVertexPositions.push_back({max.x, min.y, max.z});
        octreeVertexPositions.push_back({min.x, max.y, max.z});
        octreeVertexPositions.push_back({max.x, max.y, max.z});

        // Add the 12 faces of the bounding box
        octreeFaceIndices.push_back({startIndex + 0, startIndex + 1, startIndex + 2});
        octreeFaceIndices.push_back({startIndex + 1, startIndex + 3, startIndex + 2});
        octreeFaceIndices.push_back({startIndex + 4, startIndex + 5, startIndex + 6});
        octreeFaceIndices.push_back({startIndex + 5, startIndex + 7, startIndex + 6});
        octreeFaceIndices.push_back({startIndex + 0, startIndex + 1, startIndex + 4});
        octreeFaceIndices.push_back({startIndex + 1, startIndex + 5, startIndex + 4});
        octreeFaceIndices.push_back({startIndex + 2, startIndex + 3, startIndex + 6});
        octreeFaceIndices.push_back({startIndex + 3, startIndex + 7, startIndex + 6});
        octreeFaceIndices.push_back({startIndex + 0, startIndex + 2, startIndex + 4});
        octreeFaceIndices.push_back({startIndex + 2, startIndex + 6, startIndex + 4});
        octreeFaceIndices.push_back({startIndex + 1, startIndex + 3, startIndex + 5});
        octreeFaceIndices.push_back({startIndex + 3, startIndex + 7, startIndex + 5});
    }

    // Create the PLY file
    happly::PLYData plyOut;
    plyOut.addVertexPositions(octreeVertexPositions);
    plyOut.addFaceIndices(octreeFaceIndices);
    plyOut.write("octree.ply");

    std::cout << "Finished octree generation" << std::endl;

    return 0;
}
