cmake_minimum_required(VERSION 3.5)

project(forward_kinematics LANGUAGES CXX)

set(TEST_FILE_SOURCES
forward_kinematics_mine/forward_kinematics.cpp  Inverse_kinematics_mine/inverse_kinematics_openManipulator.cpp)

include_directories(forward_kinematics_mine
inverse_kinematics_mine)
add_compile_options(--std=c++11)
add_executable(kinematics_lib ${TEST_FILE_SOURCES})

