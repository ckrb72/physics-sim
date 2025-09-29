cmake_minimum_required(VERSION 3.20)
project(imgui)
set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")

file(GLOB_RECURSE SRC_CXX_FILES "${SOURCE_DIR}/*.cpp")


add_library(imgui ${SRC_CXX_FILES})
target_include_directories(imgui PUBLIC ${OPENGL_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../glfw/include ${CMAKE_CURRENT_SOURCE_DIR})