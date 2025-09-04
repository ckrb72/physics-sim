cmake_minimum_required(VERSION 3.20)
project(physics-sim)
set(CMAKE_CXX_STANDARD 20)


find_package(OpenGL REQUIRED)

set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
set(VENDOR_DIR "${CMAKE_SOURCE_DIR}/vendor")

include_directories(physics-sim "${VENDOR_DIR}/glad/include" "${VENDOR_DIR}/glm" "${VENDOR_DIR}/imgui")

file(GLOB_RECURSE SRC_CXX_FILES "${SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE SRC_C_FILES "${SOURCE_DIR}/*.c")

add_subdirectory(vendor/glfw)
add_subdirectory(vendor/imgui)


add_executable(physics-sim ${SRC_CXX_FILES} ${SRC_C_FILES})
target_link_libraries(physics-sim ${OPENGL_LIBRARIES} glfw imgui)
