cmake_minimum_required(VERSION 2.8)
project(learn_quaternion)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -march=native")

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} learn_quaternion.cpp)
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBS})
