cmake_minimum_required(VERSION 2.8)
project(learn_c11)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -march=native")

find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} learn_c11.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
