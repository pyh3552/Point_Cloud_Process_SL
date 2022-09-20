include_directories(code)
include_directories(code/include)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kdtree-test code/kdtree-test.cpp)
add_executable(kdtree-test-pc code/kdtree-test-pc.cpp)
add_executable(kdtree-vis code/kdtree-vis.cpp)
add_executable(octree-test code/octree-test.cpp)
add_executable(octree-easy code/octree-easy.cpp)


target_link_libraries(kdtree-test ${PCL_LIBRARIES})
target_link_libraries(kdtree-test-pc ${PCL_LIBRARIES})
target_link_libraries(kdtree-vis ${PCL_LIBRARIES})
target_link_libraries(octree-test ${PCL_LIBRARIES})