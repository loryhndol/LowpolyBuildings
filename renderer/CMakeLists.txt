cmake_minimum_required(VERSION 3.10)

project(Renderer)

file(GLOB SRC_FILES src/*.cc)
add_library(Renderer STATIC ${SRC_FILES})
target_link_libraries(Renderer PRIVATE Eigen3::Eigen glfw glad)