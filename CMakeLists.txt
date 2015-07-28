#============================================================================
#
#  Copyright 2015 by:
#
#    Commonwealth Scientific and Industrial Research Organisation (CSIRO)
#    
#    This file is licensed by CSIRO under the copy of the CSIRO Open Source
#    Software License Agreement (variation of the BSD / MIT License) included
#    with the file.
#
#    As a condition of this license, you agree that where you make any 
#    adaptations, modifications, further developments, or additional features 
#    available to CSIRO or the public in connection with your access to the 
#    Software, you do so on the terms of the BSD 3-Clause License template,
#    a copy available at: http://opensource.org/licenses/BSD-3-Clause
#
#    For further information, contact the CSIRO Workspace Team:
#    workspace@csiro.au
#
#  This copyright notice must be included with all copies of the source code.
#
#============================================================================

cmake_minimum_required(VERSION 2.8.12)

project(POINTCLOUD)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    find_path(WORKSPACE_CMAKE_ROOT Workspace_CMake_Root.txt
              HINTS ${CMAKE_SOURCE_DIR} ENV WORKSPACE_CMAKE_ROOT
              DOC "Directory containing the Workspace_CMake_Root.txt file")
    if (NOT WORKSPACE_CMAKE_ROOT)
        message(FATAL_ERROR "Please set WORKSPACE_CMAKE_ROOT to the directory containing the file Workspace_CMake_Root.txt")
    endif()
    include( ${WORKSPACE_CMAKE_ROOT}/CMakeLists.txt NO_POLICY_SCOPE )
endif()

if (NOT ${POINTCLOUD_SOURCE_DIR} STREQUAL ${POINTCLOUD_BINARY_DIR})
    include_directories(${POINTCLOUD_BINARY_DIR})
endif()

include(ExternalProject)

set(POINTCLOUD_PLUGIN_VERSION ${CSIRO_WORKSPACE_VERSION})
string(REGEX MATCH "^[0-9]+" POINTCLOUD_PLUGIN_SOVERSION ${POINTCLOUD_PLUGIN_VERSION})

find_package(Qt5Core)
find_package(Qt5Widgets)
set(QT_LIBRARIES Qt5::Core;Qt5::Widgets)

message(STATUS "If Boost detection fails as part of finding PCL, explicitly add a CMake variable for Boost_INCLUDE_DIR pointing to the version of boost being used by PCL (eg. C:/Program Files/PCL 1.6.0/3rdParty/Boost/include for the Windows All-in-one installer). This prevents the Boost version that ships with Workspace for Python support causing issues.")

find_package(PCL 1.3 REQUIRED COMPONENTS common features surface kdtree search filters)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    set(PCL_USED_LIBS ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SEARCH_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_FILTERS_LIBRARIES})
    
# Build libLAS with LASzip support if the source if available    
set(LIBLAS_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../libLAS" CACHE PATH "Location of the libLAS source code")
set(LASZIP_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../LASzip" CACHE PATH "Location of the LASzip source code")
if ((NOT EXISTS "${LIBLAS_SOURCE}/include/liblas/liblas.hpp") OR 
    (NOT EXISTS "${LASZIP_SOURCE}/include/laszip/laszip.hpp"))
    message("Unable to locate either libLAS or LASzip source code. "
            "Please set LIBLAS_SOURCE and LASZIP_SOURCE cache variables to enable this support")
	add_definitions(-DNO_LIBLAS)
    set(LIBLAS_LIBRARY "")
else()
    set(USE_LIBLAS TRUE)
    set(EXTERNAL_CMAKE_GENERATOR "Unix Makefiles")
    if (WIN32)
        set(EXTERNAL_CMAKE_GENERATOR "NMake Makefiles")
    endif()
    
    # The libs/dlls/so/dylibs won't exist until we do first compile and 
    # ExternalProject_Add makes it - hence can't use find_file/find_library.
    
    # Todo: fix this somehow or at least setup platform specific targets
    if (WIN32)
        set(LIBLAS_LIBRARY     ${CMAKE_BINARY_DIR}-libLAS/bin/RelWithDebInfo/liblas.lib)
        set(LIBLAS_DYNAMIC_LIB ${CMAKE_BINARY_DIR}-libLAS/bin/RelWithDebInfo/liblas.dll)
        set(LASZIP_LIBRARY     ${CMAKE_BINARY_DIR}-LASzip/bin/laszip.lib)    
        set(LASZIP_DYNAMIC_LIB ${CMAKE_BINARY_DIR}-LASzip/bin/laszip.dll)
    else()
        message("Need to setup libLAS/LASzip target names for this platform")
    endif()
    
    ExternalProject_Add(
        LASzip
        SOURCE_DIR ${LASZIP_SOURCE}
        CMAKE_GENERATOR ${EXTERNAL_CMAKE_GENERATOR}
        CMAKE_CACHE_ARGS -DLASZIP_BUILDOSGEO4W:BOOL=FALSE -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
        BINARY_DIR "${CMAKE_BINARY_DIR}-LASzip"
        INSTALL_COMMAND "")
                
    include_directories(${LASZIP_SOURCE}/include)
                
    ExternalProject_Add(
        libLAS
        DEPENDS LASzip
        SOURCE_DIR ${LIBLAS_SOURCE}
        CMAKE_GENERATOR ${EXTERNAL_CMAKE_GENERATOR}
        CMAKE_CACHE_ARGS 
            -DWITH_LASZIP:BOOL=TRUE 
            -DLASZIP_INCLUDE_DIR:PATH=${LASZIP_SOURCE}/include 
            -DLASZIP_LIBRARY:FILEPATH=${LASZIP_LIBRARY}
            -DBUILD_OSGEO4W:BOOL=FALSE 
            -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo 
            -DBOOST_ROOT:PATH=${Boost_INCLUDE_DIR}
        BINARY_DIR "${CMAKE_BINARY_DIR}-libLAS"
        INSTALL_COMMAND "")
        
    include_directories(${LIBLAS_SOURCE}/include)
        
endif()

# Build SMOOTH SIGNED DISTANCE SURFACE RECONSTRUCTION SOFTWARE from the ssd directory
include_directories(${POINTCLOUD_SOURCE_DIR})
include_directories(${POINTCLOUD_SOURCE_DIR}/ssd)
add_subdirectory(${POINTCLOUD_SOURCE_DIR}/ssd)

include(${POINTCLOUD_SOURCE_DIR}/src/CMakeLists.txt)

IF(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996")
ELSE()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated ") 
ENDIF()

qtx_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})
qtx_wrap_ui(UIC_SOURCES ${UI_SOURCES})
qtx_add_resources(RESOURCES resources/pointcloud_resources.qrc)

add_definitions(-DPOINTCLOUD_PLUGIN_VERSION=${POINTCLOUD_PLUGIN_VERSION})
add_library(pointcloudplugin ${SOURCES} ${HEADERS} ${MOC_SOURCES} ${UIC_SOURCES} ${RESOURCES})
message(STATUS ${LASZIP_LIBRARY})
target_link_libraries(pointcloudplugin dataanalysisplugin meshplugin workspace dgp octree ssd ${LIBLAS_LIBRARY} ${PCL_LIBRARIES} ${QT_LIBRARIES})

set_target_properties(pointcloudplugin PROPERTIES
    DEFINE_SYMBOL CSIRO_POINTCLOUD_EXPORT
    VERSION ${POINTCLOUD_PLUGIN_VERSION}
    SOVERSION ${POINTCLOUD_PLUGIN_SOVERSION}
)
setTargetOutputDirectory(pointcloudplugin ${CSIRO_INSTALL_AREA}/lib/Plugins)
configure_file(src/pkg-pointcloudplugin.cmake ${CSIRO_INSTALL_AREA}/cmake/Exports/pkg-pointcloudplugin.cmake @ONLY)

message(STATUS "Linking plugin with PCL bin directory - $ENV{PCL_ROOT}")
file(WRITE "${CSIRO_INSTALL_AREA}/installAreas/installArea-PointClouds.txt" "$ENV{PCL_ROOT}")
file(APPEND "${CSIRO_INSTALL_AREA}/installAreas/installArea-PointClouds.txt" "\n${CSIRO_INSTALL_AREA}")

if (LIBLAS_LIBRARY)
    add_custom_command(
        TARGET pointcloudplugin POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${LASZIP_DYNAMIC_LIB} ${CSIRO_INSTALL_AREA}/bin
        COMMAND ${CMAKE_COMMAND} -E copy ${LIBLAS_DYNAMIC_LIB} ${CSIRO_INSTALL_AREA}/bin)
endif()

message(STATUS "\nCopy ${CSIRO_INSTALL_AREA}/installAreas/installArea-PointClouds.txt into ${WORKSPACE_CMAKE_ROOT}/../installAreas to auto-load your plugin on startup\n")

# Copy our install headers into the install directory so that others can build against our plugin.
foreach(inFile ${INSTALL_HEADERS})
    string(REGEX REPLACE "(${POINTCLOUD_SOURCE_DIR}/src)(.*)" "${CSIRO_INSTALL_AREA}/include/PointCloud/\\2" outFile "${inFile}")
    configure_file(${inFile} ${outFile} COPYONLY)
endforeach(inFile)

