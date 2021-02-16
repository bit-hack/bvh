cmake_minimum_required(VERSION 3.0)
project(bvh)

file(GLOB SOURCE *.cpp *.h)

add_executable(bvh ${SOURCE})
