include_directories(code)
include_directories(code/include)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(dbscan_test code/Dbscan-test.cpp)
add_executable(RANSAC_test code/RANSAC-test.cpp)
target_link_libraries(dbscan_test ${PCL_LIBRARIES})
target_link_libraries(RANSAC_test ${PCL_LIBRARIES})