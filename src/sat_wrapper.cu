#include "sat_wrapper.cuh"
#include <cuda_runtime.h>
#include <vector>
#include <cfloat>

// Global pointers for GPU memory
cudaVector3D* d_triangles = nullptr;

__device__ void projectDevice(const cudaVector3D* points, int numPoints, const cudaVector3D& axis, float& min, float& max) {
    min = FLT_MAX;
    max = -FLT_MAX;
    for (int i = 0; i < numPoints; ++i) {
        float val = points[i].dot(axis);
        if (val < min) min = val;
        if (val > max) max = val;
    }
}

__global__ void intersectsKernel(const BoundingBox box, const int* triangleIndices, int numTriangles, bool* results, const cudaVector3D* d_triangles) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numTriangles) return;

    int triIdx = triangleIndices[idx];
    const cudaVector3D* tri = &d_triangles[triIdx * 3];
    cudaVector3D boxVertices[8];

    // Compute the 8 vertices of the bounding box
    for (int i = 0; i < 8; ++i) {
        boxVertices[i] = cudaVector3D(box.min.x, box.min.y, box.min.z);
        if (i & 1) boxVertices[i].x = box.max.x;
        if (i & 2) boxVertices[i].y = box.max.y;
        if (i & 4) boxVertices[i].z = box.max.z;
    }

    // Check box axes
    cudaVector3D boxNormals[3] = {
        cudaVector3D(1, 0, 0),
        cudaVector3D(0, 1, 0),
        cudaVector3D(0, 0, 1)
    };

    for (int i = 0; i < 3; ++i) {
        float boxMin, boxMax;
        projectDevice(boxVertices, 8, boxNormals[i], boxMin, boxMax);

        float triangleMin, triangleMax;
        projectDevice(tri, 3, boxNormals[i], triangleMin, triangleMax);
        if (triangleMax < boxMin || triangleMin > boxMax) {
            results[idx] = false;
            return;
        }
    }

    // Check triangle's normal
    cudaVector3D triangleEdges[3] = {
        tri[1] - tri[0],
        tri[2] - tri[1],
        tri[0] - tri[2]
    };
    cudaVector3D triangleNorm = triangleEdges[0].cross(triangleEdges[1]);
    float triangleOffset = triangleNorm.dot(tri[0]);

    float boxMin, boxMax;
    projectDevice(boxVertices, 8, triangleNorm, boxMin, boxMax);
    if (boxMax < triangleOffset || boxMin > triangleOffset) {
        results[idx] = false;
        return;
    }

    // Check cross-products of triangle edges and box normals
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cudaVector3D axis = triangleEdges[i].cross(boxNormals[j]);
            projectDevice(boxVertices, 8, axis, boxMin, boxMax);

            float triangleMin, triangleMax;
            projectDevice(tri, 3, axis, triangleMin, triangleMax);
            if (boxMax < triangleMin || boxMin > triangleMax) {
                results[idx] = false;
                return;
            }
        }
    }

    // If no separating axis is found
    results[idx] = true;
}


// After some errors, I've seen that we need to call kernels or runtime CUDA functions via wrappers in the .cu
// files. Doing this, all can be compiled without errors.

namespace Wrapper {
    void loadTrianglesToGPU(const std::vector<Triangle>& h_triangles) {
        size_t triangleSize = h_triangles.size() * 3 * sizeof(cudaVector3D);
        cudaMalloc(&d_triangles, triangleSize);

        std::vector<cudaVector3D> h_cudaTriangles;
        h_cudaTriangles.reserve(h_triangles.size() * 3);

        for (const auto& tri : h_triangles) {
            h_cudaTriangles.emplace_back(tri.vertices[0].x, tri.vertices[0].y, tri.vertices[0].z);
            h_cudaTriangles.emplace_back(tri.vertices[1].x, tri.vertices[1].y, tri.vertices[1].z);
            h_cudaTriangles.emplace_back(tri.vertices[2].x, tri.vertices[2].y, tri.vertices[2].z);
        }

        cudaMemcpy(d_triangles, h_cudaTriangles.data(), triangleSize, cudaMemcpyHostToDevice);
    }

    void freeGPUMemory() {
        cudaFree(d_triangles);
    }

    void callIntersectsKernel(const BoundingBox& box, const std::vector<int>& triangleIndices, bool*& results) {
        int numTriangles = triangleIndices.size();

        int* d_triangleIndices = nullptr;
        bool* d_results = nullptr;
        size_t indexSize = numTriangles * sizeof(int);
        size_t resultSize = numTriangles * sizeof(bool);

        cudaMalloc(&d_triangleIndices, indexSize);
        cudaMalloc(&d_results, resultSize);

        cudaMemcpy(d_triangleIndices, triangleIndices.data(), indexSize, cudaMemcpyHostToDevice);

        int blockSize = 256;
        int gridSize = (numTriangles + blockSize - 1) / blockSize;
        intersectsKernel<<<gridSize, blockSize>>>(box, d_triangleIndices, numTriangles, d_results, d_triangles);

        results = new bool[numTriangles];
        cudaMemcpy(results, d_results, resultSize, cudaMemcpyDeviceToHost);

        cudaFree(d_triangleIndices);
        cudaFree(d_results);
    }
}

