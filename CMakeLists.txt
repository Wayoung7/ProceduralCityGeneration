cmake_minimum_required(VERSION 3.15)
set (CMAKE_CXX_STANDARD 17
set(CMAKE_CXX_COMPILER "/usr/bin/g++"))
project(ProceduralCityGeneration VERSION 0.1.0)

add_subdirectory(src)
add_subdirectory(third_party)