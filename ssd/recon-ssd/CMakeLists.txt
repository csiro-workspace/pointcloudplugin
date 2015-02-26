SET(ssd_sources
    ../isosurface/MarchingCubes.hpp
    ../isosurface/MarchingCubes.inl
    ../isosurface/DualMarchingCubes.hpp
    ../isosurface/DualMarchingCubes.inl
    ../isosurface/MAT.hpp
    ../isosurface/MAT.inl
    HRTimer.hpp
    Settings.hpp
    run_ssd.hpp
    ssd_recon.cpp
)
ADD_LIBRARY(ssd STATIC ${ssd_sources})   
TARGET_LINK_LIBRARIES(ssd dgp octree)
ADD_EXECUTABLE(ssd_recon ${ssd_sources})
TARGET_LINK_LIBRARIES(ssd_recon dgp octree)
setTargetOutputDirectory(ssd_recon ${CSIRO_INSTALL_AREA}/lib)
launcher_for_executable(ssd_recon)

