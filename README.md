# MiniVR

This is the `MiniVR` project, a simple version of a volumetric renderer.
From a model in .ply format, it obtains an octree with a certain precision (depth) specified, and generates a new ply with the created octree.


## Requirements

- CMake (version 3.10 or higher)
- A C++ compiler (e.g., GCC, Clang, MSVC) with C++17 support


## Execution Instructions

Follow these steps to build and run the project:

1. **Clone the repository**:

    ```bash
    git clone https://github.com/Isma1108/MiniVR.git
    cd MiniVR
    ```

2. **Create a build directory**:

    ```bash
    mkdir build
    cd build
    ```

3. **Configure the project with CMake**:

    Run CMake to generate the build files. This could be a Makefile, a Visual Studio project, etc., depending on your platform and CMake configuration.

    ```bash
    cmake ..
    ```

4. **Build the project**:

    Once CMake has generated the build files, you can compile the project. If you are using Make, the command will be:

    ```bash
    make
    ```

    Or, on Windows with Visual Studio:

    ```bash
    cmake --build . --config Release
    ```

5. **Run the project**:

    After building, you can run the generated executable. The name and location of the executable will depend on your CMake configuration. Typically, it will be in the `build` directory.

    ```bash
    ./MiniVR
    ```
