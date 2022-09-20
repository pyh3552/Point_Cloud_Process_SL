include_directories(code)
include_directories(code/include)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(FPFH-test code/FPFH-test.cpp)
add_executable(test code/test.cpp)

target_link_libraries(FPFH-test ${PCL_LIBRARIES})