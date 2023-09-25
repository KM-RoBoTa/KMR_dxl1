cmake_minimum_required(VERSION 3.20)
project(KMR_dxlP1)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(KMR_dxlP1 
            source/KMR_dxlP1_robot.cpp
            source/KMR_dxlP1_handler.cpp
            source/KMR_dxlP1_reader.cpp
            source/KMR_dxlP1_writer.cpp
            source/KMR_dxlP1_hal.cpp)

# Directories containing header files
target_include_directories(KMR_dxlP1 PUBLIC include)

# Locations of the used libraries
target_link_directories(KMR_dxlP1 PUBLIC /usr/local/lib)

# Link the used libraries: dynamixel and yaml-cpp
target_link_libraries(KMR_dxlP1 yaml-cpp dxl_x64_cpp)

# Generate Docs
option(BUILD_DOCS "Generate Docs" ON)
if(BUILD_DOCS)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/docs)  # path to docs' cmakelists
endif()