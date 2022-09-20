cmake_minimum_required(VERSION 3.19)
project(Point_Cloud_Process_SL)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})


add_subdirectory(01-Intro_and_basic_algorithm)
add_subdirectory(02-Nearest-Neighbor-Problem)
add_subdirectory(03-Clustering)
add_subdirectory(04-Model-Fitting)
add_subdirectory(07-Feature-Detection)
add_subdirectory(08-Feature-Description)
add_subdirectory(09-Registration)
add_executable(view main.cpp)