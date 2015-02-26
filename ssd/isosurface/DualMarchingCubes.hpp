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

#ifndef __DUALMARCHINGCUBES_HPP__
#define __DUALMARCHINGCUBES_HPP__

#include <math.h>
#include <vector>

class DualMarchingCubes
{
    template <typename Octree> class Edge
    {
    public:
        typedef typename Octree::VertexKey VertexKey;
        VertexKey v1;
        VertexKey v2;
        Edge(VertexKey n1, VertexKey n2) : v1(std::min(n1,n2)), v2(std::max(n1,n2)) {}

        inline operator size_t() const {typename Octree::VertexHasher H; return H(v1);}
        inline operator size_t() {typename Octree::VertexHasher H; return H(v1);}

        inline bool operator==(const Edge& edge) const {return (v1==edge.v1 && v2==edge.v2);}
        inline bool operator!=(const Edge& edge) const {return (v1!=edge.v1 || v2!=edge.v2);}
    };

    template <typename PointType> static inline PointType RootPosition(Real isoValue, PointType const& p1, PointType const& p2, Real v1, Real v2);

public:
    template <typename Octree, typename BBox, typename PointType>
    static void getIsoSurface(Octree & octree, BBox const& bbox, Real isoValue, std::vector<PointType> & vertices, std::vector<std::vector<size_t> > & polygons, bool useFull);

    template<class Point3D>
    static void PolygonToTriangleMesh(const std::vector<Point3D> & vertices, const std::vector< std::vector<size_t> > & polygons, std::vector<std::vector<size_t> >& triangles);

    template<class Point3D>
    static void PolygonToManifoldTriangleMesh(std::vector<Point3D> & vertices, const std::vector< std::vector<size_t> > & polygons, std::vector<std::vector<size_t> >& triangles);
};

#include "DualMarchingCubes.inl"

#endif //__DUALMARCHINGCUBES_HPP__
