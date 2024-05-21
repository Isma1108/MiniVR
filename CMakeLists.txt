cmake_minimum_required(VERSION 3.10)
project(MiniVR)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SOURCES "src/*.cpp")

include_directories(include)

add_executable(MiniVR ${SOURCES})

