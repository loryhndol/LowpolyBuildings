cmake_minimum_required(VERSION 3.0)

project(Renderer)

find_package(Eigen3 REQUIRED NO_MODULE)

include(${CMAKE_SOURCE_DIR}/third-party/third-party.cmake)
file(GLOB GLAD_SRC ${GLAD_DIR}/src/glad.c)
file(GLOB SRC_FILES src/*.cc)
include_directories(include)
add_library(Renderer STATIC ${GLAD_SRC} ${SRC_FILES})
target_include_directories(Renderer PRIVATE ${GLAD_DIR}/include ${GLFW_DIR}/include)
target_link_libraries(Renderer PRIVATE Eigen3::Eigen glfw3)