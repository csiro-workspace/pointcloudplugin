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

#ifndef __POINTXYZ_HPP__
#define __POINTXYZ_HPP__

#include "inttypes.h"

template <typename Real_> class PointXYZ_
{
public:
    typedef Real_ Real;

    //coords
    union {
        Real data[4];
        struct {
            Real x;
            Real y;
            Real z;
            Real w;
        };
    };

    PointXYZ_() : x(0.f), y(0.f), z(0.f), w(0.f) {}
    PointXYZ_(Real x, Real y, Real z, Real w = 0.f) : x(x), y(y), z(z), w(w) {}
    PointXYZ_(Real scalar) : x(scalar), y(scalar), z(scalar), w(scalar) {}

    inline PointXYZ_<Real> operator+(const PointXYZ_<Real> &rhs) const {return PointXYZ_<Real>(x+rhs.x, y+rhs.y, z+rhs.z, w+rhs.w);}
    inline PointXYZ_<Real> operator-(const PointXYZ_<Real> &rhs) const {return PointXYZ_<Real>(x-rhs.x, y-rhs.y, z-rhs.z, w-rhs.w);}
    inline PointXYZ_<Real> operator*(const PointXYZ_<Real> &rhs) const {return PointXYZ_<Real>(x*rhs.x, y*rhs.y, z*rhs.z, w*rhs.w);}
    inline PointXYZ_<Real> operator/(const PointXYZ_<Real> &rhs) const {return PointXYZ_<Real>(x/rhs.x, y/rhs.y, z/rhs.z, w/rhs.w);}

    inline PointXYZ_<Real> & operator+=(const PointXYZ_<Real> &rhs) {x+=rhs.x; y+=rhs.y; z+=rhs.z; w+=rhs.w; return (*this);}
    inline PointXYZ_<Real> & operator-=(const PointXYZ_<Real> &rhs) {x-=rhs.x; y-=rhs.y; z-=rhs.z; w-=rhs.w; return (*this);}
    inline PointXYZ_<Real> & operator*=(const PointXYZ_<Real> &rhs) {x*=rhs.x; y*=rhs.y; z*=rhs.z; w*=rhs.w; return (*this);}
    inline PointXYZ_<Real> & operator/=(const PointXYZ_<Real> &rhs) {x/=rhs.x; y/=rhs.y; z/=rhs.z; w/=rhs.w; return (*this);}
};

typedef PointXYZ_<float> PointXYZ;

#endif // __POINTXYZ_HPP__
