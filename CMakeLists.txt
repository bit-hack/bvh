cmake_minimum_required(VERSION 3.0)
project(bvh)

find_package(SDL REQUIRED)
include_directories(${SDL_INCLUDE_DIR})

add_library(bvh bvh/bvh.cpp bvh/bvh.h)
add_executable(demo demo/main.cpp)
target_link_libraries(demo bvh ${SDL_LIBRARY})
