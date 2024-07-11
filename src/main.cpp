    #include <iostream>
    #include <filesystem>
    #include "happly.h"
    #include "BoundingBox.hh"
    #include "Octree.hh"

    namespace fs = std::filesystem;

    int main() {
        //happly::PLYData plyIn("mesh.ply", true);
        
        
        int depth = 8;
        /*
        std::cout << "Max Depth: ";
        std::cin >> depth;

        // Listamos todos los archivos .ply en el directorio actual, excepto octree.ply
        std::vector<std::string> plyFiles;
        for (const auto& entry : fs::directory_iterator(".")) {
            if (entry.is_regular_file() && entry.path().extension() == ".ply" && entry.path().filename() != "octree.ply") {
                plyFiles.push_back(entry.path().string());
            }
        }

        if (plyFiles.empty()) {
            std::cerr << "No .ply files found in the current directory." << std::endl;
            return 1;
        }

        std::cout << "Select a .ply file to load:" << std::endl;
        for (size_t i = 0; i < plyFiles.size(); ++i) {
            std::cout << i + 1 << ": " << plyFiles[i] << std::endl;
        }

        size_t choice = 0;
        std::cout << "Enter the number of the file to load: ";
        std::cin >> choice;

        if (choice < 1 || choice > plyFiles.size()) {
            std::cerr << "Invalid choice." << std::endl;
            return 1;
        }

        std::string selectedFile = plyFiles[choice - 1];
        std::cout << "Loading file: " << selectedFile << std::endl;
        */
        std::string selectedFile = "mesh.ply";

        // Cargamos el archivo PLY utilizando la API de happly
        happly::PLYData plyIn(selectedFile, true);

        // Cargamos la malla
        std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
        std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

        // Calculamos la bounding box global
        BoundingBox globalBounds = calculateGlobalBoundingBox(vPos);

        std::cout << "Bounding box global"<< std::endl;
        std::cout << "Min: " << globalBounds.min[0] << " " <<  globalBounds.min[1] << " " <<  globalBounds.min[2] << std::endl;
        std::cout << "Max: " << globalBounds.max[0] << " " <<  globalBounds.max[1] << " " <<  globalBounds.max[2] << std::endl;
        std:: cout << std::endl;

        Vector3D mid = (globalBounds.min + globalBounds.max) / static_cast<Real>(2.0);

        for (int i = 0; i < 8; ++i) {
            Vector3D newMin = globalBounds.min;
            Vector3D newMax = mid;

            if (i & 1) newMin.x = mid.x; else newMax.x = globalBounds.max.x;
            if (i & 2) newMin.y = mid.y; else newMax.y = globalBounds.max.y;
            if (i & 4) newMin.z = mid.z; else newMax.z = globalBounds.max.z;

            //std::cout << "Subnode " << i+1 << std::endl;
            //std::cout << "Min: " << newMin.x << " " << newMin.y << " " << newMin.z << std::endl; 
            //std::cout << "Max: " << newMax.x << " " << newMax.y << " " << newMax.z << std::endl;  

            //std::cout << std::endl; 
        }


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

            triangles.push_back(triangle);
        }


        Octree octree(globalBounds, depth);
        octree.build(triangles);

        // Colectar nodos hojas grises
        std::vector<OctreeNode*> greyLeafNodes = octree.collectGreyLeafNodes();

        // Extraer las posiciones de los vértices de los nodos hojas grises
        std::vector<std::array<double, 3>> octreeVertexPositions;
        std::vector<std::vector<size_t>> octreeFaceIndices;

        for (const auto& node : greyLeafNodes) {
            const auto& min = node->bounds.min;
            const auto& max = node->bounds.max;

            size_t startIndex = octreeVertexPositions.size();

            // Añadir los 8 vértices de la bounding box del nodo
            octreeVertexPositions.push_back({min.x, min.y, min.z});
            octreeVertexPositions.push_back({max.x, min.y, min.z});
            octreeVertexPositions.push_back({min.x, max.y, min.z});
            octreeVertexPositions.push_back({max.x, max.y, min.z});
            octreeVertexPositions.push_back({min.x, min.y, max.z});
            octreeVertexPositions.push_back({max.x, min.y, max.z});
            octreeVertexPositions.push_back({min.x, max.y, max.z});
            octreeVertexPositions.push_back({max.x, max.y, max.z});

            // Añadir las 12 caras de la bounding box
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

        // Crear el archivo PLY
        happly::PLYData plyOut;
        plyOut.addVertexPositions(octreeVertexPositions);
        plyOut.addFaceIndices(octreeFaceIndices);
        plyOut.write("octree.ply");


        std::cout << "Finished octree generation" << std::endl;

        return 0;
    }
