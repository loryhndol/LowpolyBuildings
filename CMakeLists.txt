cmake_minimum_required(VERSION 3.10)
project(LowpolyGen)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(glfw3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${GLFW3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/third-party/glad/include)

add_subdirectory(third-party)
add_subdirectory(renderer)
add_subdirectory(algorithms)
add_subdirectory(demo)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/engine/install)
install(TARGETS Engine LIBRARY DESTINATION lib)
install(FILES ${LIB_HDRS} DESTINATION include)