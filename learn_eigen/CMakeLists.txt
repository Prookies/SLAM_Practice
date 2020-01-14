cmake_minimum_required(VERSION 2.8)
project(learn_eigen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(learn_eigen_rotate learn_eigen_rotate.cpp)
