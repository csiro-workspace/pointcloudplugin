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

#include <math.h>
#include <queue>

template <typename SSDType>
OctreeBundle<SSDType>::OctreeBundle(Settings const& D) :
    D(D),
    bbox(3),
    octree(),
    points(),
    data(points),
    cache(),
    ssd(cache, bbox, data, points)
{
  ssd.set_config(D);
  data.setSamplesPerNode(D.samplesPerNode);
}

template <typename SSDType>
void OctreeBundle<SSDType>::init(void)
{
    typename Octree::cell_iterator iter = octree.cell_begin();
    if (iter!=octree.cell_end())
    {
        data.init(*iter);
    }

    adjust_bbox(true);
}

template <typename SSDType>
void OctreeBundle<SSDType>::adjust_bbox(bool cube)
{
    typename PointContainer::const_iterator iter=points.begin();
    if (iter==points.end())
    {   //no points
        return;
    }

    PointType pmin, pmax;
    pmin.x = pmax.x = iter->x;
    pmin.y = pmax.y = iter->y;
    pmin.z = pmax.z = iter->z;
    ++iter;
    while (iter!=points.end())
    {
        if (iter->x<pmin.x) {pmin.x = iter->x;}
        if (iter->y<pmin.y) {pmin.y = iter->y;}
        if (iter->z<pmin.z) {pmin.z = iter->z;}

        if (iter->x>pmax.x) {pmax.x = iter->x;}
        if (iter->y>pmax.y) {pmax.y = iter->y;}
        if (iter->z>pmax.z) {pmax.z = iter->z;}

        ++iter;
    }
    bbox.setMin(0, pmin.x);
    bbox.setMin(1, pmin.y);
    bbox.setMin(2, pmin.z);
    bbox.setMax(0, pmax.x);
    bbox.setMax(1, pmax.y);
    bbox.setMax(2, pmax.z);
    if (cube) 
    {
        double max_side = std::max(bbox.getSide(0), std::max(bbox.getSide(1), bbox.getSide(2)));
        bbox.setSide(0, max_side);
        bbox.setSide(1, max_side);
        bbox.setSide(2, max_side);
    }
}

template <typename SSDType>
size_t OctreeBundle<SSDType>::octree_refine(size_t max_level)
{
    //invalidate octree cache
    cache.clear();

    //add all cells
    std::queue<typename Octree::CellKey> queue;
    for (typename Octree::cell_iterator iter=octree.cell_begin(); iter!=octree.cell_end(); ++iter)
    {
        size_t L = octree.getCellLevel((*iter).first);
        if (L<max_level && data.test(*iter))
        {   //need split
            typename Octree::CellKey k = (*iter).first;
            queue.push(k);
        }
    }

    //traverse cell queue
    size_t splits = 0;
    while (!queue.empty())
    {
        typename Octree::CellKey key = queue.front();
        size_t L = octree.getCellLevel(key);

        //split
        typename Octree::Cell children[8];
        octree.splitCell(key, children);

        //notify octree data (split points)
        data.split(octree, bbox, key, children);

        if (L+1<max_level)
        {   //add children to queue
            for (int i=0; i<8; ++i)
            {
                if (data.test(children[i]))
                {
                    queue.push(children[i].first);
                }
            }
        }
        
        ++splits;

        //remove from queue
        queue.pop();
    }

    return splits;
}

template <typename SSDType>
void OctreeBundle<SSDType>::octree_balance(int leveldiff) 
{
    //invalidate octree cache
    cache.clear();

    bool finish = false;
    while (!finish)
    {
        finish = true;

        //add all cells to the queue
        std::queue<typename Octree::CellKey> queue;
        for (typename Octree::cell_iterator iter=octree.cell_begin(); iter!=octree.cell_end(); ++iter)
        {
            typename Octree::CellKey k = (*iter).first;
            queue.push(k);
        }

        while (!queue.empty())
        {
            typename Octree::CellKey key = queue.front(); queue.pop();
            typename Octree::CellKey cellA = octree.getCellFirstAncestor(key).first;
            if (cellA==octree.INVALID_CELL_KEY)
            {   //cell was removed
                continue;
            }

            //get neighbors
            typename Octree::CellKey neighbor_keys[6];
            octree.getFaceNeighborKeys(cellA, neighbor_keys);

            typename Octree::Cell neighbors[6] = { octree.getCellFirstAncestor(neighbor_keys[0]),
                                          octree.getCellFirstAncestor(neighbor_keys[1]),
                                          octree.getCellFirstAncestor(neighbor_keys[2]),
                                          octree.getCellFirstAncestor(neighbor_keys[3]),
                                          octree.getCellFirstAncestor(neighbor_keys[4]),
                                          octree.getCellFirstAncestor(neighbor_keys[5])};

            size_t L = octree.getCellLevel(cellA);
            for (size_t i=0; i<6; ++i)
            {
                typename Octree::CellKey cellB = neighbors[i].first;
                if (cellB==octree.INVALID_CELL_KEY)
                {
                    continue;
                }

                ptrdiff_t diff = static_cast<ptrdiff_t>(L)-octree.getCellLevel(cellB);
                if (diff > leveldiff)
                {
                    //split
                    typename Octree::Cell children[8];
                    octree.splitCell(cellB, children);

                    //notify octree data (split points)
                    data.split(octree, bbox, cellB, children);

                    for (size_t j=0; j<8; ++j)
                    {
                        queue.push(children[j].first);
                    }

                    finish = false;
                }
            }

        }

    }
}
