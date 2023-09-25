# How to setup {#setup}
[TOC]

## Dependencies
This library is dependent on the following libraries that need to be installed first:
- dynamixel: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
- yaml-cpp: https://github.com/jbeder/yaml-cpp
- CMake
- (Doxygen and Graphviz if you wish to regenerate the documentation locally)

## Installation
This library does not need to be installed per se. It has to be built into a static library (.a) in a "build" folder: 
```bash
mkdir build
cd build
cmake ../
cmake --build .
```
This will generate the static library "libKMR_dxl.a" in the "build" folder. 

In case you wish to regenerate the documentation locally, run
```bash
make docs
```
from the build folder. This will generate a "index.html" file in "docs/generated_docs/html".

## Include in a project
It is very straightforward to include this library in a project. \n
All the headers are located in "KMR_dxl/include", and, as already mentioned, the static libraries in "KMR_dxl/build".

In the root CMakeLists.txt of the project, add:
```cmake
# Path to KMR_dxl's CMakeLists
add_subdirectory(path-to-KMR_dxl)

# Path to KMR_dxl's header files
target_include_directories(project PUBLIC path-to-KMR_dxl/include)

# Path to the static (.a) library
target_link_directories(project PRIVATE path-to-KMR_dxl/build)

# Link the library
target_link_libraries(project KMR_dxl)

```

In the source code, only one header needs to be included:
```cpp
#include "KMR_dxl_robot.hpp"
```

Next: how to [use](#how-to-use)