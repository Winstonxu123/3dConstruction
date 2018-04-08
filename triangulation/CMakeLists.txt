cmake_minimum_required(VERSION 2.6)
project(triangulation)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenCV REQUIRED)

file(GLOB src_file1 "/home/xu/3weiConstruction/triangulation/triangulation.cpp")
file(GLOB src_file2 "/home/xu/3weiConstruction/triangulation/find_feature.cpp")
file(GLOB src_file3 "/home/xu/3weiConstruction/triangulation/pose_estimation_2d2d.cpp")

add_library(part SHARED ${src_file1} ${src_file2} ${src_file3})
add_executable(triangulation main.cpp)
target_link_libraries(triangulation part ${OpenCV_LIBS})
install(TARGETS triangulation RUNTIME DESTINATION bin)
