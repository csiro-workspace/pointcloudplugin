/*
Copyright (c) 2013, Daniel Moreno and Gabriel Taubin
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

#ifndef __OCTREEPOINTDATA_HPP__
#define __OCTREEPOINTDATA_HPP__

#include "OctreeDataBase.hpp"

#include <vector>

#include "SSDData.hpp"
#include "dgp/PointXYZNormal.hpp"
#include "dgp/PointXYZRGBNormal.hpp"

template <typename Octree, typename Point>
class OctreePointData : public OctreeDataBase<Octree> 
{
    std::vector<Point> const& _points;
    std::vector<size_t> _partition;
    size_t _samples;

    typedef SSDColorData<Real> Color;

    static inline void get_color(PointXYZNormal const& pt, Color & color) {}
    static inline void get_color(PointXYZRGBNormal const& pt, Color & color);

    inline void update_color(SSDCellData<Real> * cell_data, Color color) {}
    inline void update_color(SSDColorCellData<Real> * cell_data, Color color);

public:
    OctreePointData(std::vector<Point> const& points);

    void init(typename Octree::Cell cell);
    bool test(const typename Octree::Cell cell);
    void split(Octree const& octree, BBox const& bbox, typename Octree::CellKey key, typename Octree::Cell children[8]);

    inline size_t getNextPoint(size_t p) const {return _partition[p];}

    inline void setSamplesPerNode(size_t samples) {_samples=samples;}
};

#include "OctreePointData.inl"

#endif // __OCTREEPOINTDATA_HPP__
