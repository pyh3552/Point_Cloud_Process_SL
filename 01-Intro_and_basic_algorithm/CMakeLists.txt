include_directories(code)
include_directories(code/include)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${Boost_DEFINITIONS})



add_executable(pca_normal_main code/pca_normal_main.cpp code/include/pca_normal.cpp)
target_link_libraries(pca_normal_main ${PCL_LIBRARIES})

add_executable(voxel_filter_main code/voxel_filter_main.cpp code/include/voxel_filter.cpp)
target_link_libraries(voxel_filter_main ${PCL_LIBRARIES})

add_executable(bil_filter_NorBased code/bil_filter_NorBased.cpp)
target_link_libraries(bil_filter_NorBased ${PCL_LIBRARIES})

add_executable(bil-csdn code/bil-csdn.cpp)
target_link_libraries(bil-csdn ${PCL_LIBRARIES})

add_executable(add_noise code/add_noise.cpp)
target_link_libraries(add_noise ${PCL_LIBRARIES})
target_link_libraries(add_noise ${Boost_LIBRARIES})
target_link_libraries(add_noise ${Thread_LIBRARIES})