cmake_minimum_required(VERSION 3.19)
project(Point_Cloud_Process_SL)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})


add_subdirectory(01-Intro_and_basic_algorithm)

add_executable(view main.cpp)