cmake_minimum_required(VERSION 3.0)
project(Demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
find_package(OpenMP)

find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
include(${CMAKE_SOURCE_DIR}/third-party/third-party.cmake)

add_executable(Demo demo.cc)
target_include_directories(Demo PRIVATE ${GLAD_DIR}/include ${GLFW_DIR}/include ${CMAKE_SOURCE_DIR}/algorithms/include ${CMAKE_SOURCE_DIR}/renderer/include)
target_link_libraries(Demo PRIVATE CGAL::CGAL Eigen3::Eigen Engine)

if(OpenMP_CXX_FOUND)
  target_link_libraries(Demo PUBLIC OpenMP::OpenMP_CXX)
endif()
