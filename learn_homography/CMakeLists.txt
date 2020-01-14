cmake_minimum_required(VERSION 2.8)
project(learn_homography)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} virtual-billboard.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
