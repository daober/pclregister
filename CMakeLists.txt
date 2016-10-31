cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(register C CXX)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#with this option, GCC automatically optimizes for local computer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

#set(CMAKE_CXX_FLAGS "-fno-limit-debug-info")

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#headers are in include directory
include_directories(include)

set(SOURCES src/main.cpp src/registrator.cpp src/features.cpp src/loader.cpp src/filters.cpp)

add_executable(register ${SOURCES} include/registrator.hpp include/features.hpp include/loader.hpp include/filters.hpp)
target_link_libraries (register ${PCL_LIBRARIES} )
