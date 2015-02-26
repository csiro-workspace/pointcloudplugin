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

#include <cassert>

#include "Morton.hpp"
    
template <typename CellData, typename VertexData>
const size_t HashOctree<CellData,VertexData>::max_coord[23] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192,
    16384, 32768, 65536, 131072, 262144, 524288, 1048576, 2097152, 4194304};

template <typename CellData, typename VertexData>
const double HashOctree<CellData,VertexData>::cell_width[23] = {1.0, 5e-1, 2.5e-1, 1.25e-1, 6.25e-2, 3.125e-2, 1.5625e-2, 7.8125e-3,
    3.90625e-3, 1.953125e-3, 9.765625e-4, 4.8828125e-4, 2.44140625e-4, 1.220703125e-4, 6.103515625e-5, 
    3.0517578125e-5, 1.52587890625e-5, 7.62939453125e-6, 3.814697265625e-6, 1.9073486328125e-6, 
    9.5367431640625e-7, 4.76837158203125e-7, 2.384185791015625e-7}; //exact: level 22 

template <typename CellData, typename VertexData>
HashOctree<CellData,VertexData>::HashOctree() :
    ROOT(1),
    _cells(HASH_SIZE,INVALID_CELL_DATA),
    _vertices(VERTEX_HASH_SIZE,INVALID_VERTEX_DATA),
    _leaf_count(0),
    _non_leaf_count(0),
    _levels(0),
    INVALID_CELL_KEY(MAX_VALUE),
    INVALID_CELL_DATA(CellData::invalid()),
    INVALID_VERTEX_KEY(MAX_VALUE),
    INVALID_VERTEX_DATA(VertexData::invalid())
{
    //create root node
    make_root();
}

template <typename CellData, typename VertexData>
inline typename HashOctree<CellData,VertexData>::CellKey HashOctree<CellData,VertexData>::getCellKey(size_t L, size_t x, size_t y, size_t z) const
{
    assert(L>=0 && x>=0 && y>=0 && z>=0);
    return morton::make_key(x, y, z, L);
}

template <typename CellData, typename VertexData>
inline void HashOctree<CellData,VertexData>::getCellCoords(CellKey key, size_t coords[4]) const
{
    morton::split_key(key, coords[1], coords[2], coords[3], coords[0]);
}

template <typename CellData, typename VertexData>
inline size_t HashOctree<CellData,VertexData>::getCellLevel(CellKey key) const
{
    return morton::key_level(key);
}

template <typename CellData, typename VertexData>
void HashOctree<CellData,VertexData>::getCellNeighborKeys(CellKey key, const size_t * direction, CellKey * neighbors) const
{
    morton::neighbor_keys(key, direction, neighbors);
}

template <typename CellData, typename VertexData>
void HashOctree<CellData,VertexData>::make_root(void) 
{
    //create the root cell
    _cells.insert(ROOT, CellData());
    _leaf_count = 1;
    _non_leaf_count = 0;
    assert(_cells.size()==_leaf_count+_non_leaf_count);

    //create vertices
    VertexKey vertices[8];
    morton::vertices_keys(ROOT, vertices, MAX_LEVEL);
    _vertices.insert(vertices[0], 0.f);
    _vertices.insert(vertices[1], 0.f);
    _vertices.insert(vertices[2], 0.f);
    _vertices.insert(vertices[3], 0.f);
    _vertices.insert(vertices[4], 0.f);
    _vertices.insert(vertices[5], 0.f);
    _vertices.insert(vertices[6], 0.f);
    _vertices.insert(vertices[7], 0.f);
}

template <typename CellData, typename VertexData>
typename HashOctree<CellData,VertexData>::Cell HashOctree<CellData,VertexData>::getCellFirstAncestor(CellKey key)
{
    Cell cell;
    typename CellHash::Item item;
    bool found = false;
    while (key>0 && key<INVALID_CELL_KEY && !found)
    {
        item = _cells.get(key, &found);
        if (found)
        {
            cell.first = (_isnan(item.second->f) ? INVALID_CELL_KEY : item.first); //check wether is a leaf
            cell.second = item.second;
            return cell;
        }
        key >>= 3;
    }
    cell.first = INVALID_CELL_KEY; 
    return cell;
}

template <typename CellData, typename VertexData>
void HashOctree<CellData,VertexData>::getVertices(CellKey key, Vertex vertices[8])
{
    VertexKey keys[8];
    morton::vertices_keys(key, keys, MAX_LEVEL);
    vertices[0] = Vertex(keys[0], _vertices.get(keys[0]).second);
    vertices[1] = Vertex(keys[1], _vertices.get(keys[1]).second);
    vertices[2] = Vertex(keys[2], _vertices.get(keys[2]).second);
    vertices[3] = Vertex(keys[3], _vertices.get(keys[3]).second);
    vertices[4] = Vertex(keys[4], _vertices.get(keys[4]).second);
    vertices[5] = Vertex(keys[5], _vertices.get(keys[5]).second);
    vertices[6] = Vertex(keys[6], _vertices.get(keys[6]).second);
    vertices[7] = Vertex(keys[7], _vertices.get(keys[7]).second);
}

template <typename CellData, typename VertexData>
void HashOctree<CellData,VertexData>::splitCell(CellKey cell_key, Cell children[8])
{
    //find and save the current cell data
    typename CellHash::Data cell_data;

    //retrieve parent cell data and makes it non-leaf
    if (!_cells.access(cell_key, &_cells.INVALID, &cell_data)) {assert(false);}
    --_leaf_count; ++_non_leaf_count;

    //delete instead of leave it as non-leaf
    _cells.remove(cell_key); --_non_leaf_count;

    //update levels
    _levels = std::max(_levels, 1+morton::key_level(cell_key));

    //create children cells
    CellKey key[8];
    morton::children(cell_key, key);
    // WARNING: reallocationg of the hash table might invalidate previous pointers
    //    (e.g. inserting children[3] might invalidate the pointer on children[0])
    children[0] = Cell(key[0], _cells.insert(key[0], cell_data).second);
    children[1] = Cell(key[1], _cells.insert(key[1], cell_data).second);
    children[2] = Cell(key[2], _cells.insert(key[2], cell_data).second);
    children[3] = Cell(key[3], _cells.insert(key[3], cell_data).second);
    children[4] = Cell(key[4], _cells.insert(key[4], cell_data).second);
    children[5] = Cell(key[5], _cells.insert(key[5], cell_data).second);
    children[6] = Cell(key[6], _cells.insert(key[6], cell_data).second);
    children[7] = Cell(key[7], _cells.insert(key[7], cell_data).second);
    _leaf_count += 8;

    assert(_cells.size()==_leaf_count+_non_leaf_count);

    //get current cell vertices
    Vertex vertices[8];
    getVertices(cell_key, vertices);

    //save current values before modifying
    const VertexData f[8] = {*vertices[0].second, *vertices[1].second, *vertices[2].second, *vertices[3].second,
                             *vertices[4].second, *vertices[5].second, *vertices[6].second, *vertices[7].second};

    //edges
    bool found;
    typename VertexHash::Item vertex;
    vertex = _vertices.get(morton::middle_vertex(vertices[0].first, vertices[1].first), &found, true); if (!found) { *vertex.second = VertexData(f[0],f[1]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[0].first, vertices[2].first), &found, true); if (!found) { *vertex.second = VertexData(f[0],f[2]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[0].first, vertices[4].first), &found, true); if (!found) { *vertex.second = VertexData(f[0],f[4]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[1].first, vertices[3].first), &found, true); if (!found) { *vertex.second = VertexData(f[1],f[3]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[1].first, vertices[5].first), &found, true); if (!found) { *vertex.second = VertexData(f[1],f[5]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[2].first, vertices[3].first), &found, true); if (!found) { *vertex.second = VertexData(f[2],f[3]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[2].first, vertices[6].first), &found, true); if (!found) { *vertex.second = VertexData(f[2],f[6]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[3].first, vertices[7].first), &found, true); if (!found) { *vertex.second = VertexData(f[3],f[7]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[4].first, vertices[5].first), &found, true); if (!found) { *vertex.second = VertexData(f[4],f[5]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[4].first, vertices[6].first), &found, true); if (!found) { *vertex.second = VertexData(f[4],f[6]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[5].first, vertices[7].first), &found, true); if (!found) { *vertex.second = VertexData(f[5],f[7]); }
    vertex = _vertices.get(morton::middle_vertex(vertices[6].first, vertices[7].first), &found, true); if (!found) { *vertex.second = VertexData(f[6],f[7]); }

    //faces
    vertex = _vertices.get(morton::middle_vertex(vertices[0].first, vertices[3].first), &found, true);
        if (!found) { *vertex.second = VertexData(f[0],f[1],f[2],f[3]); } //0,1,2,3
    vertex = _vertices.get(morton::middle_vertex(vertices[0].first, vertices[5].first), &found, true);
        if (!found) { *vertex.second = VertexData(f[0],f[1],f[4],f[5]); } //0,1,4,5
    vertex = _vertices.get(morton::middle_vertex(vertices[0].first, vertices[6].first), &found, true);
        if (!found) { *vertex.second = VertexData(f[0],f[2],f[4],f[6]); } //0,2,4,6
    vertex = _vertices.get(morton::middle_vertex(vertices[1].first, vertices[7].first), &found, true);
        if (!found) { *vertex.second = VertexData(f[1],f[3],f[5],f[7]); } //1,3,5,7
    vertex = _vertices.get(morton::middle_vertex(vertices[2].first, vertices[7].first), &found, true);
        if (!found) { *vertex.second = VertexData(f[2],f[3],f[6],f[7]); } //2,3,6,7
    vertex = _vertices.get(morton::middle_vertex(vertices[4].first, vertices[7].first), &found, true);
        if (!found) { *vertex.second = VertexData(f[4],f[5],f[6],f[7]); } //4,5,6,7

    //center
    vertex = _vertices.get(morton::middle_vertex(vertices[0].first, vertices[7].first), &found, true);
        if (!found) { *vertex.second = VertexData(f[0],f[1],f[2],f[3],f[4],f[5],f[6],f[7]); }
}

template <typename CellData, typename VertexData>
void HashOctree<CellData,VertexData>::print_stats(void) const
{
    double cell_mean;
    size_t cell_max[2], cell_count[2];
    _cells.get_stats(cell_mean, cell_max, cell_count);
    fprintf(stdout, "Cell Hash Stats: mean=%lf, max={%lu (%lu), %lu (%lu)}\n", cell_mean, cell_max[0], cell_count[0], cell_max[1], cell_count[1]);

    double vert_mean;
    size_t vert_max[2], vert_count[2];
    _vertices.get_stats(vert_mean, vert_max, vert_count);
    fprintf(stdout, "Vertices Hash Stats: mean=%lf, max={%lu (%lu), %lu (%lu)}\n", vert_mean, vert_max[0], vert_count[0], vert_max[1], vert_count[1]);
}
