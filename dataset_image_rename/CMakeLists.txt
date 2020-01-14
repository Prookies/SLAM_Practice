cmake_minimum_required(VERSION 2.8)
project(dataset_image_rename)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -march=native")

find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image_rename rename.cpp)
target_link_libraries(image_rename ${OpenCV_LIBS})
