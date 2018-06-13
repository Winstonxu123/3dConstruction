cmake_minimum_required(VERSION 2.6)
project(icp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

file(GLOB src_file "/home/xu/3weiConstruction/lib-icp/icp/src/*.cpp")
add_library(part SHARED ${src_file})
add_executable(icp main.cpp)
target_link_libraries(icp part)

