cmake_minimum_required(VERSION 3.20)
project(physics-sim)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

find_package(OpenGL REQUIRED)

set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
set(VENDOR_DIR "${CMAKE_SOURCE_DIR}/vendor")

include_directories(physics-sim "${VENDOR_DIR}/glad/include")

file(GLOB_RECURSE SRC_CXX_FILES "${SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE SRC_C_FILES "${SOURCE_DIR}/*.c")

add_subdirectory(vendor/glfw)

FetchContent_Declare(
	glm
	GIT_REPOSITORY	https://github.com/g-truc/glm.git
	GIT_TAG 	bf71a834948186f4097caa076cd2663c69a10e1e #refs/tags/1.0.1
)

FetchContent_MakeAvailable(glm)

add_executable(physics-sim ${SRC_CXX_FILES} ${SRC_C_FILES})
target_link_libraries(physics-sim ${OPENGL_LIBRARIES} glfw glm::glm)
