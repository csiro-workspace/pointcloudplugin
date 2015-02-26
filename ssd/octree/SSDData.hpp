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

#ifndef __SSDDATA_HPP__
#define __SSDDATA_HPP__

#ifdef NO_DATA
#undef NO_DATA
#endif //NO_DATA

#include <limits>

typedef double Real;

//Octree cell data for running regular SSD
template <typename Real_> class SSDCellData
{
public:
    typedef Real_ Real;
    Real f;
    size_t first_point;

    //statistics
    size_t point_count;

    SSDCellData(SSDCellData<Real> const& other) : f(other.f), first_point(other.first_point), point_count(other.point_count) {}
    SSDCellData() : f(0), first_point(NO_DATA), point_count(0) {}
    SSDCellData(Real f, size_t first_point) : f(f), first_point(first_point), point_count(0) {}
    static inline const SSDCellData<Real> invalid(void) {return SSDCellData(std::numeric_limits<Real>::quiet_NaN(), NO_DATA);}

    template <typename PointType> void set_point_color(PointType & p) const;

    enum {NO_DATA = SIZE_MAX};
};

//Octree Vertex data for running SSD
template <typename Real_> class SSDVertexData
{
public:
    typedef Real_ Real;
    Real f;

    SSDVertexData(Real f = 0.f) : f(f) {}
    static inline SSDVertexData invalid(void) {return SSDVertexData(std::numeric_limits<Real>::quiet_NaN());}

    //building vertex from other vertices
    SSDVertexData(SSDVertexData<Real> const& v1, SSDVertexData<Real> const& v2) : f(0.5f*(v1.f+v2.f)) {}
    SSDVertexData(SSDVertexData<Real> const& v1, SSDVertexData<Real> const& v2, SSDVertexData<Real> const& v3, SSDVertexData<Real> const& v4) : f(0.25f*(v1.f+v2.f+v3.f+v4.f)) {}
    SSDVertexData(SSDVertexData<Real> const& v1, SSDVertexData<Real> const& v2, SSDVertexData<Real> const& v3, SSDVertexData<Real> const& v4,
                  SSDVertexData<Real> const& v5, SSDVertexData<Real> const& v6, SSDVertexData<Real> const& v7, SSDVertexData<Real> const& v8) : f(0.125f*(v1.f+v2.f+v3.f+v4.f+v5.f+v6.f+v7.f+v8.f)) {}
};

//Color data type
template <typename Real> class SSDColorData
{
public:
    Real r, g, b;

    SSDColorData(SSDColorData<Real> const& other) : r(other.r), g(other.g), b(other.b) {}
    SSDColorData() : r(0), g(0), b(0) {}
    SSDColorData(Real r, Real g, Real b) : r(r), g(g), b(b) {}
    SSDColorData(Real scalar) : r(scalar), g(scalar), b(scalar) {}

    inline SSDColorData<Real> operator+(const SSDColorData<Real> &rhs) const {return SSDColorData<Real>(r+rhs.r, g+rhs.g, b+rhs.b);}
    inline SSDColorData<Real> operator-(const SSDColorData<Real> &rhs) const {return SSDColorData<Real>(r-rhs.r, g-rhs.g, b-rhs.b);}
    inline SSDColorData<Real> operator*(const SSDColorData<Real> &rhs) const {return SSDColorData<Real>(r*rhs.r, g*rhs.g, b*rhs.b);}
    inline SSDColorData<Real> operator/(const SSDColorData<Real> &rhs) const {return SSDColorData<Real>(r/rhs.r, g/rhs.g, b/rhs.b);}

    inline SSDColorData<Real> & operator+=(const SSDColorData<Real> &rhs) {r+=rhs.r; g+=rhs.g; b+=rhs.b; return (*this);}
    inline SSDColorData<Real> & operator-=(const SSDColorData<Real> &rhs) {r-=rhs.r; g-=rhs.g; b-=rhs.b; return (*this);}
};

//Octree cell data for running color SSD
template <typename Real_> class SSDColorCellData
{
public:
    typedef Real_ Real;
    Real f;
    SSDColorData<Real> color;
    size_t first_point;

    //statistics
    size_t point_count;
    SSDColorData<Real> point_color_sum;

    SSDColorCellData(SSDColorCellData<Real> const& other) : f(other.f), color(other.color), first_point(other.first_point), point_count(other.point_count), point_color_sum(other.point_color_sum) {}
    SSDColorCellData() : f(0), color(), first_point(NO_DATA), point_count(0), point_color_sum() {}
    SSDColorCellData(Real f, size_t first_point) : f(f), color(), first_point(first_point), point_count(0), point_color_sum() {}
    static inline const SSDColorCellData<Real> invalid(void) {return SSDColorCellData(std::numeric_limits<Real>::quiet_NaN(), NO_DATA);}

    template <typename PointType> void set_point_color(PointType & p) const;

    enum {NO_DATA = SIZE_MAX};
};

//Octree Vertex data for running SSD
template <typename Real_> class SSDColorVertexData
{
public:
    typedef Real_ Real;
    Real f;

    SSDColorVertexData(Real f = 0.f) : f(f) {}
    static inline SSDColorVertexData invalid(void) {return SSDColorVertexData(std::numeric_limits<Real>::quiet_NaN());}

    //building vertex from other vertices
    SSDColorVertexData(SSDColorVertexData<Real> const& v1, SSDColorVertexData<Real> const& v2) : f(0.5f*(v1.f+v2.f)) {}
    SSDColorVertexData(SSDColorVertexData<Real> const& v1, SSDColorVertexData<Real> const& v2, SSDColorVertexData<Real> const& v3, SSDColorVertexData<Real> const& v4) : f(0.25f*(v1.f+v2.f+v3.f+v4.f)) {}
    SSDColorVertexData(SSDColorVertexData<Real> const& v1, SSDColorVertexData<Real> const& v2, SSDColorVertexData<Real> const& v3, SSDColorVertexData<Real> const& v4,
                       SSDColorVertexData<Real> const& v5, SSDColorVertexData<Real> const& v6, SSDColorVertexData<Real> const& v7, SSDColorVertexData<Real> const& v8) : f(0.125f*(v1.f+v2.f+v3.f+v4.f+v5.f+v6.f+v7.f+v8.f)) {}
};

#endif // __SSDDATA_HPP__
