cmake_minimum_required(VERSION 2.8)
project(loopdetection)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

# opencv
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(DBoW3_INCLUDE_DIRS "/usr/local/include")
set(DBoW3_LIBS "/usr/local/lib/libDBoW3.a")



add_executable(loopdetection main.cpp)
target_link_libraries(loopdetection ${OpenCV_LIBS} ${DBoW3_LIBS})

add_executable(CalculateSimilarity CalculateSimilarity.cpp)
target_link_libraries(CalculateSimilarity ${OpenCV_LIBS} ${DBoW3_LIBS})

#install(TARGETS loopdetection RUNTIME DESTINATION bin)
