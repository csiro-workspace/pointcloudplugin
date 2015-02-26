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

//Simplified octree interface

#ifndef __OCTREEBASE_HPP__
#define __OCTREEBASE_HPP__

#include <cstdlib>
#include <utility>

template <typename CellKey_, typename CellData_, typename VertexKey_, typename VertexData_>
class OctreeBase
{
public:
    typedef CellKey_ CellKey;
    typedef CellData_ CellData;
    typedef std::pair<CellKey,CellData*> Cell;
    typedef VertexKey_ VertexKey;
    typedef VertexData_ VertexData;
    typedef std::pair<VertexKey,VertexData*> Vertex;

    virtual size_t getLevels(void) const=0;
    virtual size_t getNumberOfCells(void) const=0;
    virtual size_t getNumberOfVertices(void) const=0;

    virtual bool isValidCellKey(CellKey key) const=0;

    // translate from cell ID to cell anchor coords (not required to check if the cell exists)
    //    coords = {L, x, y, z}
    virtual void getCellCoords(CellKey key, size_t coords[4]) const=0;

    // translate from cell coords to cell ID (not required to check if the cell exists)
    virtual CellKey getCellKey(size_t L, size_t x, size_t y, size_t z) const=0;

    // get the level of cell given its key (same as L from getCellKey(...) but possible faster)
    virtual size_t getCellLevel(CellKey key) const=0;

    // ** this is a convenient function ** get equal size face neighbor keys (not required to check if they exists)
    virtual void getFaceNeighborKeys(CellKey key, CellKey neighbors[6]) const
    {
        const size_t direction[] = {12, 14, 10, 16, 4, 22, SIZE_MAX};
        getCellNeighborKeys(key, direction, neighbors);
    }

    // get equal size neighbor keys (not required to check if they exists)
    virtual void getCellNeighborKeys(CellKey key, const size_t * direction, CellKey * neighbors) const=0;

    // get cell width for each level
    virtual double getCellWidth(size_t L) const=0;

    // Retrieve a cell corresponding to the key or its first ancestor if any found
    virtual Cell getCellFirstAncestor(CellKey key)=0;

    // get the eight vertices of a cell
    virtual void getVertices(CellKey key, Vertex vertices[8])=0;

    //subdivide
    //  children are the new cells sorted using morton key order
    virtual void splitCell(CellKey cell_key, Cell children[8])=0;

    //cell iterator
    class cell_iterator_impl
    {
    public:
        virtual Cell cell(void)=0;
        virtual void increment(void)=0;
        virtual bool equal(const cell_iterator_impl * rhs) const=0;
        virtual bool end(void) const=0;
    };
    
    class cell_iterator
    {
        cell_iterator_impl * _impl;
        inline void test_end(void) {if (_impl && _impl->end()) {delete _impl; _impl = NULL;}}
    public:
        cell_iterator(cell_iterator_impl * impl = NULL) : _impl(impl) {test_end();}
        ~cell_iterator() {if (_impl) {delete _impl;} }
        inline Cell operator*() {return _impl->cell();}
        //inline Cell * operator->() {return &(_impl->cell());}
        inline cell_iterator & operator++()
        {
            if (_impl)
            {
                _impl->increment();
            }
            test_end();
            return (*this);
        }
        inline bool operator==(const cell_iterator & rhs) {return (_impl?_impl->equal(rhs._impl):rhs._impl==NULL);}
        inline bool operator!=(const cell_iterator & rhs) {return (_impl?!_impl->equal(rhs._impl):rhs._impl!=NULL);}
    };

    virtual cell_iterator cell_begin(void)=0;
    cell_iterator cell_end(void) {return cell_iterator();}


    //vertex iterator
    class vertex_iterator_impl
    {
    public:
        virtual Vertex vertex(void)=0;
        virtual void increment(void)=0;
        virtual bool equal(const vertex_iterator_impl * rhs) const=0;
        virtual bool end(void) const=0;
    };
    
    class vertex_iterator
    {
        vertex_iterator_impl * _impl;
        inline void test_end(void) {if (_impl && _impl->end()) {delete _impl; _impl = NULL;}}
    public:
        vertex_iterator(vertex_iterator_impl * impl = NULL) : _impl(impl) {test_end();}
        ~vertex_iterator() {if (_impl) {delete _impl;} }
        inline Vertex operator*() {return (*_impl).vertex();}
        //inline Vertex * operator->() {return &((*_impl).vertex());}
        inline vertex_iterator & operator++()
        {
            if (_impl)
            {
                _impl->increment();
            }
            test_end();
            return (*this);
        }
        inline bool operator==(const vertex_iterator & rhs) {return (_impl?_impl->equal(rhs._impl):rhs._impl==NULL);}
        inline bool operator!=(const vertex_iterator & rhs) {return (_impl?!_impl->equal(rhs._impl):rhs._impl!=NULL);}
    };

    virtual vertex_iterator vertex_begin(void)=0;
    vertex_iterator vertex_end(void) {return vertex_iterator();}
};

#endif // __OCTREEBASE_HPP__
