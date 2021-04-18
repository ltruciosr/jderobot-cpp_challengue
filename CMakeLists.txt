cmake_minimum_required(VERSION 2.8)
project(test2-cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#########
# BUILD #
#########
add_executable(test2_graphs ${PROJECT_SOURCE_DIR}/test2_graphs.cpp)