cmake_minimum_required(VERSION 2.6)
project(featureextraction)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#opencv
find_package(OpenCV REQUIRED)

file(GLOB src_file1 "/home/xu/3weiConstruction/featureExtraction/find_feature.cpp")
file(GLOB src_file2 "/home/xu/3weiConstruction/featureExtraction/pose_estimation_2d2d.cpp")
add_library( part SHARED ${src_file1} ${src_file2})

add_executable(featureExtraction main.cpp)
target_link_libraries(featureExtraction ${OpenCV_LIBS} part)
install(TARGETS featureExtraction RUNTIME DESTINATION bin)
