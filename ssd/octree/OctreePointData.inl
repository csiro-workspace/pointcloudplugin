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

#include <algorithm>

template <typename T> class RangeGenerator {
public:
    RangeGenerator(T i = T()) : i(i) {}
    inline T operator()() {return (i++);};
    T i;
};

template <typename Octree, typename Point>
OctreePointData<Octree,Point>::OctreePointData(std::vector<Point> const& points) :
    _points(points), 
    _partition(1, Octree::CellData::NO_DATA),
    _samples(1)
{
}

template <typename Octree, typename Point>
inline void OctreePointData<Octree,Point>::init(typename Octree::Cell cell) 
{
    cell.second->point_count = _points.size();
    _partition.resize(cell.second->point_count, Octree::CellData::NO_DATA);
    if (cell.second->point_count>0)
    {
        std::generate_n(_partition.begin(), _partition.size()-1, RangeGenerator<size_t>(1));
        cell.second->first_point = 0;
    }
}

template <typename Octree, typename Point>
inline bool OctreePointData<Octree,Point>::test(const typename Octree::Cell cell)
{
    return (cell.second && cell.second->point_count>_samples);
}

template <typename Octree, typename Point> 
inline void OctreePointData<Octree,Point>::get_color(PointXYZRGBNormal const& pt, Color & color)
{
    color = Color(pt.r, pt.g, pt.b);
}

template <typename Octree, typename Point> 
inline void OctreePointData<Octree,Point>::update_color(SSDColorCellData<Real> * cell_data, Color color_sum)
{
    cell_data->point_color_sum = color_sum;
    if (cell_data->point_count)
    {
        cell_data->color = color_sum/Color(Real(cell_data->point_count));
    }
}

template <typename Octree, typename Point>
void OctreePointData<Octree,Point>::split(Octree const& octree, BBox const& bbox, typename Octree::CellKey key, typename Octree::Cell children[8])
{
    if (children[0].second->first_point==Octree::CellData::NO_DATA)
    {   //shortcut: no data, nothing to do
        return;
    }

    //split cell key
    size_t cell_coords[4]; octree.getCellCoords(key, cell_coords);
    const double cell_width = octree.getCellWidth(cell_coords[0]);

    //get cube center coords
    const double x = bbox.getMin(0) + bbox.getSide(0)*cell_width*(cell_coords[1]+0.5);
    const double y = bbox.getMin(1) + bbox.getSide(1)*cell_width*(cell_coords[2]+0.5);
    const double z = bbox.getMin(2) + bbox.getSide(2)*cell_width*(cell_coords[3]+0.5);

    typename Octree::CellData * data[8] = {children[0].second, children[1].second, children[2].second, children[3].second,
                                  children[4].second, children[5].second, children[6].second, children[7].second};

    Color color_sum[8];

    size_t first_point = data[0]->first_point;

    data[0]->first_point = Octree::CellData::NO_DATA;
    data[1]->first_point = Octree::CellData::NO_DATA;
    data[2]->first_point = Octree::CellData::NO_DATA;
    data[3]->first_point = Octree::CellData::NO_DATA;
    data[4]->first_point = Octree::CellData::NO_DATA;
    data[5]->first_point = Octree::CellData::NO_DATA;
    data[6]->first_point = Octree::CellData::NO_DATA;
    data[7]->first_point = Octree::CellData::NO_DATA;

    data[0]->point_count = 0;
    data[1]->point_count = 0;
    data[2]->point_count = 0;
    data[3]->point_count = 0;
    data[4]->point_count = 0;
    data[5]->point_count = 0;
    data[6]->point_count = 0;
    data[7]->point_count = 0;

    for (size_t iP=first_point, iPnext=Octree::CellData::NO_DATA; iP!=Octree::CellData::NO_DATA; iP=iPnext) 
    {
        Point const& pt = _points[iP];

        int code = 0;
        if (pt.x>x) {code |= 0x1;}
        if (pt.y>y) {code |= 0x2;}
        if (pt.z>z) {code |= 0x4;}

        iPnext = _partition[iP];
        _partition[iP] = data[code]->first_point;
        data[code]->first_point = iP;

        Color point_color;;
        get_color(pt, point_color);
        color_sum[code] += point_color;

        ++(data[code]->point_count);
    }

    update_color(data[0], color_sum[0]);
    update_color(data[1], color_sum[1]);
    update_color(data[2], color_sum[2]);
    update_color(data[3], color_sum[3]);
    update_color(data[4], color_sum[4]);
    update_color(data[5], color_sum[5]);
    update_color(data[6], color_sum[6]);
    update_color(data[7], color_sum[7]);
}
