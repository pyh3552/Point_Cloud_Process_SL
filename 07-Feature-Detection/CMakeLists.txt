include_directories(code)
include_directories(code/include)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ISS-test code/ISS-test.cpp)

target_link_libraries(ISS-test ${PCL_LIBRARIES})