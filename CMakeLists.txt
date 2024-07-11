cmake_minimum_required(VERSION 3.10)
project(MiniVR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SOURCES "src/*.cpp")

include_directories(include)

add_executable(MiniVR ${SOURCES})

find_package(OpenMP REQUIRED)

if(OpenMP_CXX_FOUND)
    target_link_libraries(MiniVR PUBLIC OpenMP::OpenMP_CXX)
endif()

