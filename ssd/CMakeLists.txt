PROJECT(ssd)
cmake_minimum_required(VERSION 2.8.12)
cmake_policy(SET CMP0020 NEW)

add_definitions(-D__STDC_LIMIT_MACROS)

if (MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

INCLUDE_DIRECTORIES(${ssd_SOURCE_DIR})
INCLUDE_DIRECTORIES(${ssd_SOURCE_DIR}/..) # seems to be a mix of #include paths (some with ssd prefix some without)

set (CMAKE_INSTALL_PREFIX "${ssd_BINARY_DIR}/bin")
SUBDIRS(dgp isosurface octree recon-ssd)
