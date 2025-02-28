# Scopic Test Task - Eberty Alves

## **Overview**

This project implements a test task for a **2D/3D Geometry Algorithm Developer position at Scopic**. The task required the development of a C++ project using modern techniques to design a `Contour` class capable of storing a series of segments, where each segment can be either a straight line or a circular arc. The solution includes:

- A polymorphic hierarchy for segments (lines and arcs) with deep copy and move semantics.
- A utility function to create contours from a series of points (interpreting them as a polyline).
- A connectivity validation method (`isValid()`) for checking if all segments are sequentially connected.
- Asynchronous tasks to separate and test valid and invalid contours.
- Basic visualization using SFML to render contours.

**Author:** Eberty Alves da Silva, <eberty.silva@hotmail.com>

**Keywords:** C++, 2D/3D Geometry, CMake, SFML, Contour, Algorithms.

&nbsp;

## **Install Dependencies**

There are no additional dependencies to install manually, as the package includes all the necessary libraries. Just ensure that you have CMake and a C++ compiler installed on your system. For visualization, the project uses [SFML](https://www.sfml-dev.org/), which is automatically fetched during the CMake configuration process, this may take a while.

## **Building and Running the Project**

Clone the repository and build the project using CMake:

```sh
git clone https://github.com/Eberty/Scopic.git
cd Scopic/
mkdir build
cd build
```

### On Linux

```sh
cmake ..
make -j$(nproc)
./scopic_test_task
```

### On Windows

```sh
cmake .. -G "Visual Studio 17 2022" # Adjust the generator to match your Visual Studio version
cmake --build . --config Release
.\Release\scopic_test_task.exe
```

Or, alternatively, you can open this folder in Microsoft Visual Studio and build the project from there.
