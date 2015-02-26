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

#include "MarchingCubes.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <functional>
#include <unordered_map>

#include "MAT.hpp"

#if defined(_MSC_VER) && (_MSC_VER<1600)
    // VS2008 defines std::unordered_map as std::tr1::unordered_map
    namespace std {using namespace std::tr1;};
#endif

template <typename PointType>
inline PointType DualMarchingCubes::RootPosition(Real isoValue, PointType const& p1, PointType const& p2, Real v1, Real v2)
{
    Real t = (v1-isoValue)/(v1-v2);
    return p1*static_cast<typename PointType::Real>(1.f-t)+p2*static_cast<typename PointType::Real>(t);
}

template <typename Octree, typename BBox, typename PointType>
void DualMarchingCubes::getIsoSurface(Octree & octree, BBox const& bbox, Real isoValue, std::vector<PointType> & vertices, std::vector<std::vector<size_t> > & polygons, bool useFull)
{
    MarchingCubes::SetCaseTable();
    MarchingCubes::SetFullCaseTable();

    static const size_t dir_neigh[9] = { 13, // Direction ( 0, 0, 0)
                                          4, // Direction (-1, 0, 0)  [*]
                                         10, // Direction ( 0,-1, 0)  [*]
                                          1, // Direction (-1,-1, 0)
                                         12, // Direction ( 0, 0,-1)  [*]
                                          3, // Direction (-1, 0,-1)
                                          9, // Direction ( 0,-1,-1)
                                          0, // Direction (-1,-1,-1)
                                          SIZE_MAX  //list end
                                        };

    std::unordered_map<Edge<Octree>,size_t,typename Octree::VertexHasher> vTable;

    //traverse vertices
    const size_t max_level = octree.getLevels();
    const size_t max_coord = size_t(1)<<max_level;
    for (typename Octree::vertex_iterator iter=octree.vertex_begin(); iter!=octree.vertex_end(); ++iter)
    {
        const typename Octree::VertexKey vkey = (*iter).first;
        size_t v[4]; octree.getCellCoords(vkey, v);
        size_t shift = v[0]-max_level-1;
        v[0] = max_level; v[1] >>= shift; v[2] >>= shift; v[3] >>= shift;
        if ( v[1]==0 || v[1]>=max_coord || v[2]==0 || v[2]>=max_coord || v[3]==0 || v[3]>=max_coord)
        {   //border vertex, no cell, skip
            continue;
        }
        const typename Octree::CellKey key = octree.getCellKey(v[0], v[1], v[2], v[3]);

        //get dual cell
        typename Octree::CellKey neighbor_keys[10];
        octree.getCellNeighborKeys(key, dir_neigh, neighbor_keys);

        typename Octree::Cell neighbors[8] = { octree.getCellFirstAncestor(neighbor_keys[0]),
                                               octree.getCellFirstAncestor(neighbor_keys[1]),
                                               octree.getCellFirstAncestor(neighbor_keys[2]),
                                               octree.getCellFirstAncestor(neighbor_keys[3]),
                                               octree.getCellFirstAncestor(neighbor_keys[4]),
                                               octree.getCellFirstAncestor(neighbor_keys[5]),
                                               octree.getCellFirstAncestor(neighbor_keys[6]),
                                               octree.getCellFirstAncestor(neighbor_keys[7]) };
        bool isDualCell = true;
        PointType center[8];
        Real values[8];
        for (size_t i=0; i<8; ++i) 
        {
            const typename Octree::CellKey nkey = neighbors[i].first;
            if (nkey==octree.INVALID_CELL_KEY)
            {
                isDualCell = false;
                break;
            }

            size_t cell_a[4]; octree.getCellCoords(nkey, cell_a);
            const double da = octree.getCellWidth(cell_a[0]);
            PointType & p = center[i];
            p.x = static_cast<typename PointType::Real>(bbox.getMin(0)+(cell_a[1]+0.5)*da*bbox.getSide(0));
            p.y = static_cast<typename PointType::Real>(bbox.getMin(1)+(cell_a[2]+0.5)*da*bbox.getSide(1));
            p.z = static_cast<typename PointType::Real>(bbox.getMin(2)+(cell_a[3]+0.5)*da*bbox.getSide(2));

            neighbors[i].second->set_point_color(p);

            values[i] = neighbors[i].second->f;
        }
        if (!isDualCell)
        {
            continue;
        }

        
        int idx;
        if (useFull)
        {
            idx = MarchingCubes::GetFullIndex(values, isoValue);
        }
        else
        {
            idx = MarchingCubes::GetIndex(values, isoValue);
        }

        // Add the necessary vertices
        std::vector<std::vector<int> > const& table = MarchingCubes::caseTable(idx, useFull);
        for (size_t i=0; i<table.size(); ++i)
        {
            std::vector<size_t> polygon;
            for(size_t j=0; j<table[i].size(); ++j)
            {
                int c1, c2;
                Cube::EdgeCorners(table[i][j], c1, c2);
                Edge<Octree> edge(neighbors[c1].first, neighbors[c2].first);
                if (vTable.find(edge)==vTable.end())
                {
                    vTable[edge] = vertices.size();
                    PointType p = RootPosition(isoValue, center[c1], center[c2], values[c1], values[c2]);
                    vertices.push_back(p);
                }
                size_t v = vTable[edge];
                if (std::find(polygon.begin(), polygon.end(), v)==polygon.end())
                {   //not found: add
                    polygon.push_back(v);
                }
            }
            if (polygon.size()>2)
            {
                polygons.push_back(polygon);
            }
        }
    }
}

template<class Point3D>
void DualMarchingCubes::PolygonToTriangleMesh(const std::vector<Point3D> & vertices, const std::vector<std::vector<size_t> > & polygons, std::vector<std::vector<size_t> > & triangles)
{
    MinimalAreaTriangulation<Point3D> mat;
    triangles.clear();
    for (size_t i=0; i<polygons.size(); ++i)
    {
        std::vector<Point3D> loop;
        std::vector<size_t> vertexMap;
        std::vector<TriangleIndex> tgl;
        loop.resize(polygons[i].size());
        vertexMap.resize(polygons[i].size());
        for (size_t j=0; j<polygons[i].size(); ++j)
        {
            loop[j] = vertices[polygons[i][j]];
            vertexMap[j] = polygons[i][j];
        }
        mat.GetTriangulation(loop, tgl);

        size_t tSize = triangles.size();
        triangles.resize(tSize+tgl.size());
        for (size_t j=0; j<tgl.size(); ++j)
        {
            triangles[tSize+j].resize(3);
            for (size_t k=0; k<3; ++k)
            {
                triangles[tSize+j][k] = vertexMap[tgl[j].idx[k]];
            }
        }
    }
}

template<class Point3D>
void DualMarchingCubes::PolygonToManifoldTriangleMesh(std::vector<Point3D> & vertices, const std::vector<std::vector<size_t> > & polygons, std::vector<std::vector<size_t> > & triangles )
{
    std::vector<size_t> t;
    t.resize( 3 );
    triangles.clear();
    for(size_t i=0 ; i<polygons.size() ; i++)
    {
        if( polygons[i].size()==3 )
            triangles.push_back( polygons[i] );
        else if( polygons[i].size()>3 )
        {
            Point3D center;
            center *= 0;
            for (size_t j=0 ; j<polygons[i].size() ; j++ ) center += vertices[ polygons[i][j] ];
            center /= static_cast<Real>(polygons[i].size());

            size_t idx = vertices.size();
            vertices.push_back( center );
            t[2] = idx;
            for(size_t j=0 ; j<polygons[i].size() ; j++ )
            {
                t[0] = polygons[i][j];
                t[1] = polygons[i][(j+1)%polygons[i].size()];
                triangles.push_back( t );
            }
        }
    }
}
