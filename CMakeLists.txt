cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(register C CXX)

SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#headers are in include directory
include_directories(include)

add_executable(register src/main.cpp src/register.cpp)
target_link_libraries (register ${PCL_LIBRARIES} )
