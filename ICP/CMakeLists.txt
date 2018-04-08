cmake_minimum_required(VERSION 2.6)
project(icp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_BUILD_TYPE "Release")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${CSPARSE_INCLUDE_DIR} "/usr/include/eigen3/")
file(GLOB src_file "/home/xu/3weiConstruction/ICP/src/*.cpp")
add_library(part SHARED ${src_file})

add_executable(icp main.cpp)
target_link_libraries(icp ${OpenCV_LIBS} part ${CSPARSE_LIBRARY} g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)

install(TARGETS icp RUNTIME DESTINATION bin)
