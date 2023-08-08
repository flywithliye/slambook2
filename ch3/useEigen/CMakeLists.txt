cmake_minimum_required(VERSION 3.0)
project(useEigen)

set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
include_directories("/usr/local/include/eigen3")
add_executable(eigenMatrix eigenMatrix.cpp)
