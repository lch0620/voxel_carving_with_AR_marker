cmake_minimum_required(VERSION 3.0.0)
project(voxel_carving_with_AR_marker VERSION 0.1.0)
set(CMAKE_GENERATOR_TOOLSET "v142" CACHE STRING "Platform Toolset")
set(OpenCV_DIR C:/Users/holde/Desktop/opencv/build)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(voxel_carving_with_AR_marker main.cpp)

target_link_libraries( voxel_carving_with_AR_marker ${OpenCV_LIBS} )