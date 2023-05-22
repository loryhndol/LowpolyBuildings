cmake_minimum_required(VERSION 3.0)
project(LowpolyGen)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "GLFW lib only")
set(GLFW_INSTALL OFF CACHE BOOL "GLFW lib only")
add_subdirectory(third-party/glfw-3.3.7)

add_subdirectory(renderer)
add_subdirectory(algorithms)
add_subdirectory(demo)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/engine/install)
install(TARGETS Engine LIBRARY DESTINATION lib)
install(FILES ${LIB_HDRS} DESTINATION include)