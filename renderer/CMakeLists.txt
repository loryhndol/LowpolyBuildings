cmake_minimum_required(VERSION 3.0)

project(Renderer)

find_package(Eigen3 REQUIRED NO_MODULE)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "GLFW lib only")
set(GLFW_INSTALL OFF CACHE BOOL "GLFW lib only")

set(GLAD_DIR ../third-party/glad)
set(GLFW_DIR ../third-party/glfw-3.3.7)
file(GLOB GLAD_SRC ${GLAD_DIR}/src/glad.c)
file(GLOB SRC_FILES src/*.cc)
file(GLOB SRC_HDRS include/*.h)

add_library(Renderer STATIC ${GLAD_SRC} ${SRC_FILES} ${SRC_HDRS})

target_include_directories(Renderer PRIVATE ${GLAD_DIR}/include ${GLFW_DIR}/include)
target_link_libraries(Renderer PRIVATE Eigen3::Eigen glfw3)