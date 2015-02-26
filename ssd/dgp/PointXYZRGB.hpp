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

#ifndef __POINTXYZRGB_HPP__
#define __POINTXYZRGB_HPP__

#include "PointXYZ.hpp"

template <typename Real> class PointXYZRGB_ : public PointXYZ_<Real>
{
public:
    //color
    union {
        Real rgba[4];
        struct {
            Real b;
            Real g;
            Real r;
            Real a;
        };
    };

    //operations from PointXYZ_
    PointXYZRGB_() : PointXYZ_<Real>(), b(0.f), g(0.f), r(0.f), a(1.f) {}
    PointXYZRGB_(Real x, Real y, Real z, Real w, Real b, Real g, Real r, Real a) : PointXYZ_<Real>(x, y, z, w), b(b), g(g), r(r), a(a) {}
    PointXYZRGB_(Real scalar) : PointXYZ_<Real>(scalar), b(scalar), g(scalar), r(scalar), a(scalar) {}

    inline PointXYZRGB_ operator+(const PointXYZRGB_ &rhs) const {return PointXYZRGB_(this->x+rhs.x, this->y+rhs.y, this->z+rhs.z, this->w+rhs.w, b+rhs.b, g+rhs.g, r+rhs.r, a+rhs.a);}
    inline PointXYZRGB_ operator-(const PointXYZRGB_ &rhs) const {return PointXYZRGB_(this->x-rhs.x, this->y-rhs.y, this->z-rhs.z, this->w-rhs.w, b-rhs.b, g-rhs.g, r-rhs.r, a-rhs.a);}
    inline PointXYZRGB_ operator*(const PointXYZRGB_ &rhs) const {return PointXYZRGB_(this->x*rhs.x, this->y*rhs.y, this->z*rhs.z, this->w*rhs.w, b*rhs.b, g*rhs.g, r*rhs.r, a*rhs.a);}
    inline PointXYZRGB_ operator/(const PointXYZRGB_ &rhs) const {return PointXYZRGB_(this->x/rhs.x, this->y/rhs.y, this->z/rhs.z, this->w/rhs.w, b/rhs.b, g/rhs.g, r/rhs.r, a/rhs.a);}

    inline PointXYZRGB_ & operator+=(const PointXYZRGB_ &rhs) {this->x+=rhs.x; this->y+=rhs.y; this->z+=rhs.z; this->w+=rhs.w; b+=rhs.b; g+=rhs.g; r+=rhs.r; a+=rhs.a; return (*this);}
    inline PointXYZRGB_ & operator-=(const PointXYZRGB_ &rhs) {this->x-=rhs.x; this->y-=rhs.y; this->z-=rhs.z; this->w-=rhs.w; b-=rhs.b; g-=rhs.g; r-=rhs.r; a-=rhs.a; return (*this);}
    inline PointXYZRGB_ & operator*=(const PointXYZRGB_ &rhs) {this->x*=rhs.x; this->y*=rhs.y; this->z*=rhs.z; this->w*=rhs.w; b*=rhs.b; g*=rhs.g; r*=rhs.r; a*=rhs.a; return (*this);}
    inline PointXYZRGB_ & operator/=(const PointXYZRGB_ &rhs) {this->x/=rhs.x; this->y/=rhs.y; this->z/=rhs.z; this->w/=rhs.w; b/=rhs.b; g/=rhs.g; r/=rhs.r; a/=rhs.a; return (*this);}
};

typedef PointXYZRGB_<float> PointXYZRGB;

#endif // __POINTXYZRGB_HPP__
