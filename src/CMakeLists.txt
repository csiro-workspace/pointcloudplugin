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

set(HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/iterativeclosestpoint.h
    ${CMAKE_CURRENT_LIST_DIR}/smoothesigneddistance.h
    ${CMAKE_CURRENT_LIST_DIR}/radiusoutlierremoval.h
    ${CMAKE_CURRENT_LIST_DIR}/mergenearbynodes.h
    ${CMAKE_CURRENT_LIST_DIR}/organizedsurfacereconstruction.h
    ${CMAKE_CURRENT_LIST_DIR}/unorganizedsurfacereconstruction.h
    ${CMAKE_CURRENT_LIST_DIR}/removestatisticaloutliers.h
    ${CMAKE_CURRENT_LIST_DIR}/movingleastsquares.h
    ${CMAKE_CURRENT_LIST_DIR}/mlsvoxel.h
    ${CMAKE_CURRENT_LIST_DIR}/mlssample.h
    ${CMAKE_CURRENT_LIST_DIR}/mlsrandomuniform.h
    ${CMAKE_CURRENT_LIST_DIR}/detectholes.h
    ${CMAKE_CURRENT_LIST_DIR}/normalestimation.h
    # Excluded for now as prebuilt PCL io module adds a Qt4 dependency 
    #${CMAKE_CURRENT_LIST_DIR}/pcdreader.h
    ${CMAKE_CURRENT_LIST_DIR}/pclmeshmodelinterface.h
    ${CMAKE_CURRENT_LIST_DIR}/pclmeshnodesinterface.h
    ${CMAKE_CURRENT_LIST_DIR}/poissonreconstruction.h
    ${CMAKE_CURRENT_LIST_DIR}/pointcloudplugin_api.h
    ${CMAKE_CURRENT_LIST_DIR}/pointcloudplugin.h
)

# Don't really need to worry about INSTALL_HEADERS as this project is available to be built from source
set(INSTALL_HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/pointcloudplugin_api.h
    ${CMAKE_CURRENT_LIST_DIR}/pointcloudplugin.h
)

set(MOC_HEADERS
)

set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/iterativeclosestpoint.cpp
    ${CMAKE_CURRENT_LIST_DIR}/smoothesigneddistance.cpp
    ${CMAKE_CURRENT_LIST_DIR}/radiusoutlierremoval.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pointcloudplugin.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mergenearbynodes.cpp
    ${CMAKE_CURRENT_LIST_DIR}/organizedsurfacereconstruction.cpp
    ${CMAKE_CURRENT_LIST_DIR}/unorganizedsurfacereconstruction.cpp
    ${CMAKE_CURRENT_LIST_DIR}/removestatisticaloutliers.cpp    
    ${CMAKE_CURRENT_LIST_DIR}/movingleastsquares.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mlsvoxel.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mlssample.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mlsrandomuniform.cpp
    ${CMAKE_CURRENT_LIST_DIR}/detectholes.cpp
    ${CMAKE_CURRENT_LIST_DIR}/normalestimation.cpp
    # Excluded for now as prebuilt PCL io module adds a Qt4 dependency 
    #${CMAKE_CURRENT_LIST_DIR}/pcdreader.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pclmeshmodelinterface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pclmeshnodesinterface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/poissonreconstruction.cpp
)

if (USE_LIBLAS)
    list(APPEND HEADERS
         ${CMAKE_CURRENT_LIST_DIR}/lasreader.h)
         
    list(APPEND SOURCES
         ${CMAKE_CURRENT_LIST_DIR}/lasreader.cpp)
endif()

set(UI_SOURCES
)
