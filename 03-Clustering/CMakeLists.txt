include_directories(code)
include_directories(code/include)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(K-means-test code/K-means-test.cpp)
add_executable(GMM-test code/GMM-test.cpp)
add_executable(Sepctral_Clustering_test code/Sepctral_Clustering_test.cpp)
add_executable(K-means-K-test code/K-means-K-test.cpp)


target_link_libraries(K-means-test ${PCL_LIBRARIES})
target_link_libraries(K-means-K-test ${PCL_LIBRARIES})
target_link_libraries(GMM-test ${PCL_LIBRARIES})
target_link_libraries(Sepctral_Clustering_test ${PCL_LIBRARIES})