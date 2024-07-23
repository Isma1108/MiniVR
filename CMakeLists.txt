cmake_minimum_required(VERSION 3.10)
project(MiniVR LANGUAGES CXX CUDA)  # Indicamos que el proyecto utiliza CUDA y C++

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find packages
find_package(OpenMP REQUIRED)
find_package(CUDA REQUIRED)

# Include directories
include_directories(include ${CUDA_INCLUDE_DIRS})  # Incluir el directorio 'include' y los directorios de inclusión de CUDA

# CUDA architecture settings (adjust as needed)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-std=c++11)
set(CUDA_LIBRARIES PUBLIC ${CUDA_LIBRARIES})  # Hack to ensure proper linkage

# C++ source files
file(GLOB CPP_SOURCES src/*.cpp)

# CUDA source files
file(GLOB CUDA_SOURCES src/*.cu)

# Add executable
add_executable(MiniVR ${CPP_SOURCES} ${CUDA_SOURCES})

# CUDA separable compilation
set_target_properties(MiniVR PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Link libraries
target_link_libraries(MiniVR PUBLIC OpenMP::OpenMP_CXX ${CUDA_LIBRARIES})  # Use PUBLIC here as suggested





