/*
Copyright (c) 2013, Daniel Moreno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//Bundle of Octree, BoundingBox, and OctreeData

#ifndef __OCTREEBUNDLE_HPP__
#define __OCTREEBUNDLE_HPP__

#include <cstdlib>

#include "dgp/BBox.h"
#include "dgp/PointXYZ.hpp"
#include "dgp/PointXYZRGB.hpp"
#include "SSDData.hpp"
#include "OctreePointData.hpp"
#include "HashOctree.hpp"
#include "OctreeCache.hpp"
#include "SmoothSignedDistance.hpp"
#include "ColorSmoothSignedDistance.hpp"

// Bundle of data types for computing regular SSD
struct SSDType
{
    typedef PointXYZNormal PointType;
    typedef PointXYZ OutPointType;
    typedef HashOctree<SSDCellData<Real>,SSDVertexData<Real> > OctreeType;
    typedef OctreePointData<OctreeType,PointType> OctreeDataType;
    typedef std::vector<PointType> PointContainer;
    typedef OctreeCache<OctreeType> CacheType;
    typedef SmoothSignedDistance<CacheType,OctreeDataType,PointType> SmoothSignedDistanceType;
};

// Bundle of data types for computing color SSD
struct SSDColorType
{
    typedef PointXYZRGBNormal PointType;
    typedef PointXYZRGB OutPointType;
    typedef HashOctree<SSDColorCellData<Real>,SSDColorVertexData<Real> > OctreeType;
    typedef OctreePointData<OctreeType,PointType> OctreeDataType;
    typedef std::vector<PointType> PointContainer;
    typedef OctreeCache<OctreeType> CacheType;
    typedef ColorSmoothSignedDistance<CacheType,OctreeDataType,PointType> SmoothSignedDistanceType;
};

template <typename SSDType>
class OctreeBundle
{
public:
    typedef typename SSDType::OctreeType Octree;
    typedef typename SSDType::PointType PointType;
    typedef typename SSDType::PointContainer PointContainer;
    typedef typename SSDType::OctreeDataType OctreeDataType;
    typedef typename SSDType::OutPointType OutPointType;
    typedef typename SSDType::CacheType CacheType;
    typedef typename SSDType::SmoothSignedDistanceType SmoothSignedDistanceType;

    Settings const& D;
    BBox bbox;
    Octree octree;
    PointContainer points;
    OctreeDataType data;
    CacheType cache;
    SmoothSignedDistanceType ssd;

    OctreeBundle(Settings const& D);

    void init(void);
    void adjust_bbox(bool cube);
    size_t octree_refine(size_t max_level);
    void octree_balance(int leveldiff);
};

#include "OctreeBundle.inl"

#endif // __OCTREEBUNDLE_HPP__
