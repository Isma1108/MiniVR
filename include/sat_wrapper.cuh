#pragma once

#include "BoundingBox.hh"
#include <cuda_runtime.h>
#include <vector>

struct cudaVector3D {
    float x, y, z;

    __device__ cudaVector3D() : x(0), y(0), z(0) {}
    __device__ cudaVector3D(float x, float y, float z) : x(x), y(y), z(z) {}

    __device__ cudaVector3D operator-(const cudaVector3D& other) const {
        return cudaVector3D(x - other.x, y - other.y, z - other.z);
    }

    __device__ float dot(const cudaVector3D& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    __device__ cudaVector3D cross(const cudaVector3D& other) const {
        return cudaVector3D(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }
};

__device__ void projectDevice(const cudaVector3D* points, int numPoints, const cudaVector3D& axis, float& min, float& max);

__global__ void intersectsKernel(const BoundingBox box, const int* triangleIndices, int numTriangles, bool* results, const cudaVector3D* d_triangles);

namespace Wrapper {
    void loadTrianglesToGPU(const std::vector<Triangle>& h_triangles);
    void freeGPUMemory();
    void callIntersectsKernel(const BoundingBox& box, const std::vector<int>& triangleIndices, bool*& results);
}
