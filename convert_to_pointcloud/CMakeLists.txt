cmake_minimum_required(VERSION 2.6)
project(convert_to_pointcloud)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#opencv
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(convert_to_pointcloud main.cpp)
target_link_libraries(convert_to_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
install(TARGETS convert_to_pointcloud RUNTIME DESTINATION bin)
