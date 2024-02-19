cmake_minimum_required(VERSION 3.10)

project(Engine)


file(GLOB LIB_SOURCE src/*.cc src/*.hpp)
add_library(Engine SHARED ${LIB_SOURCE})

include_directories(include)
target_include_directories(Engine PRIVATE ${CMAKE_SOURCE_DIR}/renderer/include)
target_link_libraries(Engine PRIVATE CGAL::CGAL Eigen3::Eigen Renderer glfw glad)
