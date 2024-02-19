cmake_minimum_required(VERSION 3.10)
project(Demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

add_executable(Demo demo.cc)
target_include_directories(Demo PRIVATE ${CMAKE_SOURCE_DIR}/algorithms/include ${CMAKE_SOURCE_DIR}/renderer/include)
target_link_libraries(Demo PRIVATE CGAL::CGAL Engine glfw glad Eigen3::Eigen)

if(OpenMP_CXX_FOUND)
  target_link_libraries(Demo PUBLIC OpenMP::OpenMP_CXX)
endif()
