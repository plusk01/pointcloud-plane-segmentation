cmake_minimum_required(VERSION 3.1)
project(rspd)

add_library(rspd SHARED src/planedetector.cpp src/plane.cpp src/planarpatch.cpp src/connectivitygraph.cpp)
target_link_libraries(rspd PUBLIC Open3D::Open3D)
target_include_directories(rspd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)