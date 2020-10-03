cmake_minimum_required(VERSION 3.1)
project(rspd)

# If you haven't installed Open3D system-wide, you will need to set the
# CMAKE_PREFIX_PATH=<install_dir>, e.g.,
# cmake -DCMAKE_PREFIX_PATH=/home/plusk01/dev/o3d/Open3D/build/install ..
find_package(Open3D)
find_package(Qt5 COMPONENTS Widgets Core)



add_library(rspd SHARED src/planedetector.cpp src/plane.cpp src/planarpatch.cpp src/connectivitygraph.cpp)
target_link_libraries(rspd PUBLIC Open3D::Open3D)
target_link_libraries(rspd PUBLIC Qt5::Widgets Qt5::Core)
target_include_directories(rspd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)