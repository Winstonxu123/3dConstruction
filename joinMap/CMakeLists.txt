cmake_minimum_required(VERSION 2.6)
project(joinmap)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#opencv
find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

#pcl 
find_package(PCL REQUIRED COMPONENT common io filters)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(joinmap main.cpp)
target_link_libraries(joinmap ${OpenCV_LIBS} ${PCL_LIBRARIES})

install(TARGETS joinmap RUNTIME DESTINATION bin)
