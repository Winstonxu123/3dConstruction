cmake_minimum_required(VERSION 2.6)
project(vo)

add_executable(vo main.cpp)

install(TARGETS vo RUNTIME DESTINATION bin)
