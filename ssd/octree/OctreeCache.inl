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

#if defined(_MSC_VER) && (_MSC_VER<1600)
    // VS2008 defines std::unordered_map as std::tr1::unordered_map
    namespace std {using namespace std::tr1;};
#endif

template <typename Octree> void OctreeCache<Octree>::clear(void)
{
    _cells.clear();
    _vertices.clear();
    _cellmap.clear();
    _vertexmap.clear();
}

template <typename Octree> void OctreeCache<Octree>::build(Octree & octree)
{
    static const size_t MORTON_TO_TAUBIN_VERTEX_ORDER[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };
                                                          // 0  1  2  3  4  5  6  7
    _octree = &octree;

    size_t cell_count = octree.getNumberOfCells();
    size_t vertex_count = octree.getNumberOfVertices();

    _cells.clear();
    _cells.resize(cell_count);
    _vertices.clear();
    _vertices.resize(vertex_count);

    _cellmap.clear();
    _cellmap.rehash(cell_count);
    _vertexmap.clear();
    _vertexmap.rehash(vertex_count);

    size_t index = 0;
    size_t v_index = 0;
    for (typename Octree::cell_iterator iter=octree.cell_begin(); iter!=octree.cell_end(); ++iter)
    {
        //original key
        typename Octree::CellKey cellA = (*iter).first;

        //mapped key
        size_t index_a;
        typename CellMap::const_iterator map_a = _cellmap.find(cellA);
        if (map_a==_cellmap.end())
        {   //not found: create
            _cellmap[cellA] = index;
            index_a = index;
            ++index;
        }
        else
        {   //found
            index_a = map_a->second;
        }

        //cache
        CellCache & cache_a = _cells[index_a];
        cache_a.key = cellA;

        //save user data (first point in partition)
        //cache_a.data = iter->second->data;
        cache_a.data = (*iter).second;

        //save cell coords
        octree.getCellCoords(cellA, cache_a.cell);

        //make neighbor edges
        typename Octree::CellKey neighbor_keys[6];
        octree.getFaceNeighborKeys(cellA, neighbor_keys);

        typename Octree::Cell neighbors[6] = { octree.getCellFirstAncestor(neighbor_keys[0]),
                                               octree.getCellFirstAncestor(neighbor_keys[1]),
                                               octree.getCellFirstAncestor(neighbor_keys[2]),
                                               octree.getCellFirstAncestor(neighbor_keys[3]),
                                               octree.getCellFirstAncestor(neighbor_keys[4]),
                                               octree.getCellFirstAncestor(neighbor_keys[5])};

        std::vector<size_t> & edges_a = cache_a.edges;
        for (size_t i=0; i<6; ++i)
        {
            typename Octree::CellKey cellB = neighbors[i].first;
            if (cellB==octree.INVALID_CELL_KEY)
            {
                continue;
            }

            //mapped key
            size_t index_b;
            typename CellMap::const_iterator map_b = _cellmap.find(cellB);
            if (map_b==_cellmap.end())
            {   //not found: create
                _cellmap[cellB] = index;
                index_b = index;
                ++index;
            }
            else
            {   //found
                index_b = map_b->second;
            }

            if (cellB<cellA)
            {
                if (std::find(edges_a.begin(), edges_a.end(), index_b)==edges_a.end())
                {
                    edges_a.push_back(index_b); 
                }
            }
            else
            {
                std::vector<size_t> & edges_b = _cells[index_b].edges;
                if (std::find(edges_b.begin(), edges_b.end(), index_a)==edges_b.end())
                {
                    edges_b.push_back(index_a);
                }
            }
        }

        //get current cell vertices
        typename Octree::Vertex vertices[8];
        octree.getVertices(cellA, vertices);
        size_t * vertices_a = cache_a.vertices;
        for (size_t i=0; i<8; ++i)
        {
            //original key
            typename Octree::VertexKey v = vertices[i].first;

            //mapped key
            size_t j;
            typename VertexMap::const_iterator map_v = _vertexmap.find(v);
            if (map_v==_vertexmap.end())
            {   //not found: add
                _vertexmap[v] = v_index;
                _vertices[v_index] = vertices[i].second;
                j = v_index;
                ++v_index;
            }
            else
            {   //found
                j = map_v->second;
            }
            
            //save in the current cell
            vertices_a[MORTON_TO_TAUBIN_VERTEX_ORDER[i]] = j;
        }

    }//for each cell
}

template <typename Octree> void OctreeCache<Octree>::getFaceNeighborKeys(CellKey key, CellKey neighbors[6]) const
{
    std::vector<size_t> const& edges = _cells[key].edges;
    std::vector<size_t>::const_iterator iter = edges.begin();
    for (size_t i=0; i<6; ++i)
    {
        neighbors[i] = (iter==edges.end() ? INVALID_CELL_KEY : *iter++);
    }
}

template <typename Octree> void OctreeCache<Octree>::getCellNeighborKeys(CellKey key, const size_t * direction, CellKey * neighbors) const
{
    typename Octree::CellKey original_neighbors[27];
    _octree->getCellNeighborKeys(_cells[key].key, direction, original_neighbors);
    for (size_t i=0; original_neighbors[i]!=MAX_VALUE; ++i)
    {
        neighbors[i] = _cellmap.find(original_neighbors[i])->second;
    }
}

template <typename Octree> typename OctreeCache<Octree>::Cell OctreeCache<Octree>::getCellFirstAncestor(CellKey key)
{   //this is just exists, we cannot do better
    //assert(false); //just a warning abut the incomplete implementation here
    if (key<_cells.size())
    {
        return Cell(key, _cells[key].data);
    }
    //not found
    return Cell();
}

template <typename Octree> void OctreeCache<Octree>::getVertices(CellKey key, Vertex vertices[8])
{
    if (key<_cells.size())
    {
        const size_t * v = _cells[key].vertices;
        vertices[0] = Vertex(v[0], _vertices[v[0]]);
        vertices[1] = Vertex(v[1], _vertices[v[1]]);
        vertices[2] = Vertex(v[2], _vertices[v[2]]);
        vertices[3] = Vertex(v[3], _vertices[v[3]]);
        vertices[4] = Vertex(v[4], _vertices[v[4]]);
        vertices[5] = Vertex(v[5], _vertices[v[5]]);
        vertices[6] = Vertex(v[6], _vertices[v[6]]);
        vertices[7] = Vertex(v[7], _vertices[v[7]]);
    }
    //not found
}
