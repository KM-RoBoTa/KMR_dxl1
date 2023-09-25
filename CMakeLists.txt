cmake_minimum_required(VERSION 3.20)
project(KMR_dxl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(KMR_dxl 
            source/KMR_dxl_robot.cpp
            source/KMR_dxl_handler.cpp
            source/KMR_dxl_reader.cpp
            source/KMR_dxl_writer.cpp
            source/KMR_dxl_hal.cpp)

# Directories containing header files
target_include_directories(KMR_dxl PUBLIC include)

# Locations of the used libraries
target_link_directories(KMR_dxl PUBLIC /usr/local/lib)

# Link the used libraries: dynamixel and yaml-cpp
target_link_libraries(KMR_dxl yaml-cpp dxl_x64_cpp)

# Generate Docs
option(BUILD_DOCS "Generate Docs" ON)
if(BUILD_DOCS)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/docs)  # path to docs' cmakelists
endif()