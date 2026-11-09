cmake_minimum_required(VERSION 3.10)
project(robot_arm_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(robot_arm robot_arm.cpp)

target_compile_options(robot_arm PRIVATE -Wall -Wextra)
