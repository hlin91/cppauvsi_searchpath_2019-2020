# CMake最低版本要求
cmake_minimum_required (VERSION 2.8)
# 项目信息
project (CppUAV)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

# 制定生成目标
add_executable(CppUAV main.cpp)
