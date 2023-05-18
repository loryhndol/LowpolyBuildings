cmake_minimum_required(VERSION 3.0)

project(Engine)

set(CMAKE_BUILD_TYPE "Release")

find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

file(GLOB LIB_SOURCE src/*.cc)
file(GLOB LIB_HDRS include/engine/*.h)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(Engine SHARED ${LIB_SOURCE})

target_link_libraries(Engine PRIVATE CGAL::CGAL Eigen3::Eigen)
