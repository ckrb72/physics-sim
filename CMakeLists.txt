cmake_minimum_required(VERSION 3.20)
project(physics-sim)
set(CMAKE_CXX_STANDARD 20)


find_package(OpenGL REQUIRED)

set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
set(VENDOR_DIR "${CMAKE_SOURCE_DIR}/vendor")

file(GLOB_RECURSE SRC_PHYSICS_FILES "${SOURCE_DIR}/physics/*.cpp")
file(GLOB_RECURSE SRC_RENDER_FILES "${SOURCE_DIR}/render/*.cpp")
file(GLOB_RECURSE SRC_RENDER_C_FILES "${SOURCE_DIR}/render/*.c")
file(GLOB_RECURSE SRC_UTIL_FILES "${SROUCE_DIR}/util/*.cpp")

add_subdirectory(vendor/glfw)
add_subdirectory(vendor/imgui)


add_executable(bouncing_sphere "${SOURCE_DIR}/apps/bouncing_sphere.cpp" ${SRC_PHYSICS_FILES} ${SRC_RENDER_FILES} ${SRC_RENDER_C_FILES} ${SRC_UTIL_FILES})
target_include_directories(bouncing_sphere PUBLIC "${VENDOR_DIR}/glad/include" "${VENDOR_DIR}/glm" "${SOURCE_DIR}")
target_link_libraries(bouncing_sphere ${OPENGL_LIBRARIES} glfw imgui)

add_executable(spline "${SOURCE_DIR}/apps/spline.cpp" ${SRC_PHYSICS_FILES} ${SRC_RENDER_FILES} ${SRC_RENDER_C_FILES} ${SRC_UTIL_FILES})
target_include_directories(spline PUBLIC "${VENDOR_DIR}/glad/include" "${VENDOR_DIR}/glm" "${SOURCE_DIR}")
target_link_libraries(spline ${OPENGL_LIBRARIES} glfw imgui)

add_executable(constraints "${SOURCE_DIR}/apps/constraints.cpp" ${SRC_PHYSICS_FILES} ${SRC_RENDER_FILES} ${SRC_RENDER_C_FILES} ${SRC_UTIL_FILES})
target_include_directories(constraints PUBLIC "${VENDOR_DIR}/glad/include" "${VENDOR_DIR}/glm" "${SOURCE_DIR}")
target_link_libraries(constraints ${OPENGL_LIBRARIES} glfw imgui)

add_executable(obb "${SOURCE_DIR}/apps/obb.cpp" ${SRC_PHYSICS_FILES} ${SRC_RENDER_FILES} ${SRC_RENDER_C_FILES} ${SRC_UTIL_FILES})
target_include_directories(obb PUBLIC "${VENDOR_DIR}/glad/include" "${VENDOR_DIR}/glm" "${SOURCE_DIR}")
target_link_libraries(obb ${OPENGL_LIBRARIES} glfw imgui)