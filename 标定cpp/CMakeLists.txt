cmake_minimum_required(VERSION 3.1)
project( navigation)

set (CMAKE_BUILD_TYPE "Release")

include_directories( "/usr/include/eigen3" )

add_executable(acc 1.cpp)
add_executable(gyo 2.cpp)
add_executable(cepiao 3.cpp)
add_executable(wucha 4.cpp)
add_executable(5wucha 5.cpp)
