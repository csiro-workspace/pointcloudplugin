SET(octree_sources 
    OctreeBase.hpp
    HashOctree.hpp
    HashOctree.inl
    OctreeDataBase.hpp
    OctreePointData.hpp
    OctreePointData.inl
    OctreeBundle.hpp
    OctreeBundle.inl
    SimpleHash.hpp
    Morton.hpp
    Morton.cpp
    SSDData.hpp
    SmoothSignedDistance.hpp
    SmoothSignedDistance.inl
    ColorSmoothSignedDistance.hpp
    ColorSmoothSignedDistance.inl
    OctreeCache.hpp
    OctreeCache.inl
)   
ADD_LIBRARY(octree STATIC ${octree_sources})
TARGET_LINK_LIBRARIES(octree dgp)
