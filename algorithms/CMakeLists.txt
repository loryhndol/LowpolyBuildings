cmake_minimum_required(VERSION 3.0)

project(Engine)

set(CMAKE_BUILD_TYPE "Release")

find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

file(GLOB LIB_SOURCE src/*.cc src/*.hpp)
add_library(Engine SHARED ${LIB_SOURCE})

include_directories(include)
include(${CMAKE_SOURCE_DIR}/third-party/third-party.cmake)
target_include_directories(Engine PRIVATE ${GLAD_DIR}/include ${GLFW_DIR}/include ${CMAKE_SOURCE_DIR}/renderer/include)
target_link_libraries(Engine PRIVATE CGAL::CGAL Eigen3::Eigen Renderer)
