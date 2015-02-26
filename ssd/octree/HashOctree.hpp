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

#ifndef __HASHOCTREE_HPP__
#define __HASHOCTREE_HPP__

#include "OctreeBase.hpp"
#include "SimpleHash.hpp"

#if !defined(_MSC_VER) && !defined(_isnan)
#   include <cmath>
#   define _isnan isnan
#endif

template <typename CellData, typename VertexData>
class HashOctree : public OctreeBase<uint64_t, CellData, uint64_t, VertexData>
{
public:
    typedef OctreeBase<uint64_t, CellData, uint64_t, VertexData> Parent;
    typedef typename Parent::CellKey CellKey;
    typedef typename Parent::CellData Celldata;
    typedef typename Parent::Cell Cell;
    typedef typename Parent::VertexKey VertexKey;
    typedef typename Parent::VertexData Vertexdata;
    typedef typename Parent::Vertex Vertex;

private:
    static const size_t HASH_BITS = 18;
    static const size_t HASH_SIZE = (1<<HASH_BITS) ;
    static const size_t VERTEX_HASH_BITS = 2+HASH_BITS;
    static const size_t VERTEX_HASH_SIZE = (1<<VERTEX_HASH_BITS) ;
    static const size_t MAX_LEVEL = 16;

    static const size_t max_coord[23];
    static const double cell_width[23];

    CellKey ROOT;

    typedef SimpleHash<CellKey,CellData,BitHash<HASH_BITS,0> > CellHash;
    typedef SimpleHash<VertexKey,VertexData,BitHash<VERTEX_HASH_BITS,24> > VertexHash; 

    CellHash _cells;
    VertexHash _vertices;
    size_t _leaf_count;
    size_t _non_leaf_count;
    size_t _levels;

public:
    const CellKey INVALID_CELL_KEY;
    const CellData INVALID_CELL_DATA;
    const VertexKey INVALID_VERTEX_KEY;
    const VertexData INVALID_VERTEX_DATA;
    
    HashOctree();

    inline size_t getLevels(void) const {return _levels;}
    inline size_t getNumberOfCells(void) const {return _leaf_count;}
    inline size_t getNumberOfVertices(void) const {return _vertices.size();}

    inline virtual bool isValidCellKey(CellKey key) const {return (key!=INVALID_CELL_KEY);}

    inline CellKey getCellKey(size_t L, size_t x, size_t y, size_t z) const;
    inline void getCellCoords(CellKey key, size_t coords[4]) const;
    inline size_t getCellLevel(CellKey key) const;
    inline void getCellNeighborKeys(CellKey key, const size_t * direction, CellKey * neighbors) const;
    inline double getCellWidth(size_t L) const {return cell_width[L];}

    Cell getCellFirstAncestor(CellKey key);

    void getVertices(CellKey key, Vertex vertices[8]);

    void splitCell(CellKey cell_id, Cell children[8]);

    //debug
    void print_stats(void) const;

    //cell iterator
    typedef typename Parent::cell_iterator_impl cell_iterator_impl;
    typedef typename Parent::cell_iterator cell_iterator;
    class hash_cell_iterator : public cell_iterator_impl
    {
        typename CellHash::iterator _curr;
        typename CellHash::iterator _end;
        inline void skip_non_leaves(void) {while (_curr!=_end && _isnan((*_curr).second->f)) {++_curr;}}
    public:
        hash_cell_iterator(const typename CellHash::iterator & curr, const typename CellHash::iterator & end) : _curr(curr), _end(end) {skip_non_leaves();}
        inline Cell cell(void) { return Cell((*_curr).first, (*_curr).second);}
        inline void increment(void) {++_curr; skip_non_leaves();}
        inline bool equal(const cell_iterator_impl * rhs) const {const hash_cell_iterator * iter = dynamic_cast<const hash_cell_iterator *>(rhs); return (iter && _curr==iter->_curr);}
        inline bool end(void) const {return (_curr==_end);}
    };
    cell_iterator cell_begin(void) {return cell_iterator(new hash_cell_iterator(_cells.begin(), _cells.end()));}

    //vertex iterator
    typedef typename Parent::vertex_iterator_impl vertex_iterator_impl;
    typedef typename Parent::vertex_iterator vertex_iterator;
    class hash_vertex_iterator : public vertex_iterator_impl
    {
        typename VertexHash::iterator _curr;
        typename VertexHash::iterator _end;
    public:
        hash_vertex_iterator(const typename VertexHash::iterator & curr, const typename VertexHash::iterator & end) : _curr(curr), _end(end) {}
        inline Vertex vertex(void) { return Vertex((*_curr).first, (*_curr).second);}
        inline void increment(void) {++_curr;}
        inline bool equal(const vertex_iterator_impl * rhs) const {const hash_vertex_iterator * iter = dynamic_cast<const hash_vertex_iterator *>(rhs); return (iter && _curr==iter->_curr);}
        inline bool end(void) const {return (_curr==_end);}
    };
    vertex_iterator vertex_begin(void) {return vertex_iterator(new hash_vertex_iterator(_vertices.begin(), _vertices.end()));}

    template <typename Key> class Hasher : public std::unary_function<Key, size_t>
    {
    public:
        typedef Key _Kty;
        typedef uint32_t _Inttype;    // use first 2*32 bits
        size_t operator()(const Key& _Keyval) const
        {
            return (std::hash<_Inttype>()((_Inttype)(_Keyval & 0xffffffffUL))^std::hash<_Inttype>()((_Inttype)(_Keyval >> 32)));
        }
    };

    typedef Hasher<CellKey> CellHasher;
    typedef Hasher<VertexKey> VertexHasher;
private:
    void make_root(void);
};

#include "HashOctree.inl"

#endif // __HASHOCTREE_HPP__
