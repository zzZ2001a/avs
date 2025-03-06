cmake_minimum_required(VERSION 3.12)
project(MCEM CXX C)

message("*****************avs_mesh_compression****************")
set (ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include(dependencies/cmake/draco.cmake)
add_subdirectory(dependencies/hpm-HPM-15.0)
add_subdirectory(source/encode)
add_subdirectory(source/decode)

