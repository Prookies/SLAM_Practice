CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本

PROJECT(PointCloudSmooth) #设定工程名
SET( CMAKE_CXX_FLAGS "-std=c++11" )#设定编译器
SET(CMAKE_BUILD_TYPE Debug)	
	
#增加子文件夹，也就是进入源代码文件夹继续构建
ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)
