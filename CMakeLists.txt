cmake_minimum_required(VERSION 3.10)
project(o3dapp)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_BUILD_TYPE Debug)


# If you haven't installed Open3D system-wide, you will need to set the
# CMAKE_PREFIX_PATH=<install_dir>, e.g.,
# cmake -DCMAKE_PREFIX_PATH=/home/plusk01/dev/o3d/Open3D/build/install ..
find_package(Open3D)
find_package(OpenMP)
add_subdirectory(rspd)

get_target_property(LIBB_INCLUDES rspd INCLUDE_DIRECTORIES)
message(STATUS "rspd includes: ${LIBB_INCLUDES}")

# Open3D
if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION} at ${Open3D_CONFIG}")
    add_executable(main src/main.cpp)
    target_link_libraries(main PRIVATE Open3D::Open3D rspd)
    if(OpenMP_CXX_FOUND)
      target_link_libraries(main PRIVATE OpenMP::OpenMP_CXX)
    endif()
else()
    message(SEND_ERROR "Open3D not found")
endif()
