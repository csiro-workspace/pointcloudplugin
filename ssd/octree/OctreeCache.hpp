/*
Copyright (c) 2013, Fatih Calakli and Daniel Moreno
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

#ifndef __OCTREECACHE_HPP__
#define __OCTREECACHE_HPP__

#include "OctreeBase.hpp"

#include <functional>
#include <unordered_map>

#if defined(_MSC_VER) && (_MSC_VER<1600)
    // VS2008 defines std::unordered_map as std::tr1::unordered_map
    namespace std {using namespace std::tr1;};
#endif

template <typename Octree>
class OctreeCache : public OctreeBase<size_t, typename Octree::CellData, size_t, typename Octree::VertexData>
{
public:
    typedef OctreeBase<size_t, typename Octree::CellData, size_t, typename Octree::VertexData> Parent;
    typedef typename Parent::CellKey CellKey;
    typedef typename Parent::CellData CellData;
    typedef typename Parent::Cell Cell;
    typedef typename Parent::VertexKey VertexKey;
    typedef typename Parent::VertexData VertexData;
    typedef typename Parent::Vertex Vertex;

private:
    typedef std::unordered_map<typename Octree::CellKey,CellKey,typename Octree::CellHasher> CellMap;
    typedef std::unordered_map<typename Octree::VertexKey,VertexKey,typename Octree::VertexHasher> VertexMap;

    struct CellCache
    {
        typename Octree::CellKey key; //original key
        CellData * data;
        std::vector<CellKey> edges;
        VertexKey vertices[8];
        size_t cell[4]; //L,i,j,k
    };
    Octree * _octree;
    std::vector<CellCache> _cells;
    std::vector<VertexData*> _vertices;
    CellMap _cellmap;
    VertexMap _vertexmap;

public:
    typedef typename std::vector<CellCache>::const_iterator cell_const_iterator;

    const CellKey INVALID_CELL_KEY;
    const VertexKey INVALID_VERTEX_KEY;
    OctreeCache() : INVALID_CELL_KEY(SIZE_MAX), INVALID_VERTEX_KEY(SIZE_MAX) {} 

    void build(Octree & octree);
    void clear(void);

    //Octree interface
    inline size_t getLevels(void) const {return _octree->getLevels();}
    inline size_t getNumberOfCells(void) const {return _cells.size();}
    inline size_t getNumberOfVertices(void) const {return _vertices.size();}

    inline bool isValidCellKey(CellKey key) const {return (key<_cells.size());}

    // translate from cell ID to cell anchor coords (not required to check if the cell exists)
    //    coords = {L, x, y, z}
    inline void getCellCoords(CellKey key, size_t coords[4]) const { memcpy(coords, _cells[key].cell, 4*sizeof(size_t)); }

    // translate from cell coords to cell ID (not required to check if the cell exists)
    inline CellKey getCellKey(size_t L, size_t x, size_t y, size_t z) const { return _cellmap.find(_octree->getCellKey(L, x, y, z))->second; }

    // get the level of cell given its key (same as L from getCellKey(...) but possible faster)
    inline size_t getCellLevel(CellKey key) const {return _cells[key].cell[0];}

    // get equal size face neighbor keys (not required to check if they exists)
    void getFaceNeighborKeys(CellKey key, CellKey neighbors[6]) const;

    // get equal size neighbor keys (not required to check if they exists)
    void getCellNeighborKeys(CellKey key, const size_t * direction, CellKey * neighbors) const;

    // get cell width for each level
    inline double getCellWidth(size_t L) const {return _octree->getCellWidth(L);}

    // Retrieve a cell corresponding to the key or its first ancestor if any found
    Cell getCellFirstAncestor(CellKey key);

    // get the eight vertices of a cell
    void getVertices(CellKey key, Vertex vertices[8]);

    //subdivide
    //  children are the new cells sorted using morton key order
    virtual void splitCell(CellKey cell_key, Cell children[8]) {}

    //cell iterator
    typedef typename Parent::cell_iterator_impl cell_iterator_impl;
    typedef typename Parent::cell_iterator cell_iterator;
    class cache_cell_iterator : public cell_iterator_impl
    {
        CellCache * _cell;
        size_t _curr;
        size_t _end;
    public:
        cache_cell_iterator(std::vector<CellCache> & cells) : _cell((cells.begin()!=cells.end()?&*cells.begin():NULL)), _curr(0), _end(cells.size()) {}
        inline Cell cell(void) { return Cell(_curr, _cell->data);}
        inline void increment(void) {++_cell; ++_curr;}
        inline bool equal(const cell_iterator_impl * rhs) const {const cache_cell_iterator * iter = dynamic_cast<const cache_cell_iterator *>(rhs); return (iter && _cell==iter->_cell);}
        inline bool end(void) const {return (_curr==_end);}
    };
    cell_iterator cell_begin(void) {return cell_iterator(new cache_cell_iterator(_cells));}

    //vertex iterator
    typedef typename Parent::vertex_iterator_impl vertex_iterator_impl;
    typedef typename Parent::vertex_iterator vertex_iterator;
    class cache_vertex_iterator : public vertex_iterator_impl
    {
        VertexData ** _vertex;
        size_t _curr;
        size_t _end;
    public:
        cache_vertex_iterator(std::vector<VertexData*> & vertices) : _vertex((vertices.begin()!=vertices.end()?&*vertices.begin():NULL)), _curr(0), _end(vertices.size()) {}
        inline Vertex vertex(void) { return Vertex(_curr, *_vertex);}
        inline void increment(void) {++_vertex; ++_curr;}
        inline bool equal(const vertex_iterator_impl * rhs) const {const cache_vertex_iterator * iter = dynamic_cast<const cache_vertex_iterator *>(rhs); return (iter && _vertex==iter->_vertex);}
        inline bool end(void) const {return (_curr==_end);}
    };
    vertex_iterator vertex_begin(void) {return vertex_iterator(new cache_vertex_iterator(_vertices));}
};

#include "OctreeCache.inl"

#endif // __OCTREECACHE_HPP__
