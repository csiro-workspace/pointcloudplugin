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

#ifndef __COLORSMOOTHSIGNEDDISTANCE_HPP__
#define __COLORSMOOTHSIGNEDDISTANCE_HPP__

#include <vector>
#include <numeric>

template <typename Octree, typename OctreeDataType, typename PointType>
class ColorSmoothSignedDistance 
{
    Octree & _octree;
    BBox const& _bbox;
    OctreeDataType const& _data;
    std::vector<PointType> const& _points;

    double _lambda1;
    double _lambda2;
    double _lambda3;
    double _lambda1_color;
    double _lambda2_color;
    double _tol;
    double _tol_color;
    size_t _miniter;
    size_t _maxiter;
    double _normalizer1;
    double _normalizer2;
    double _normalizer3;
    double _normalizer1_color;
    double _normalizer2_color;

    static const double grad[3][8];

    typedef SSDColorData<double> Color;

    void accumulate_normalizers(void);
    void accumulate(std::vector<double> & residual, std::vector<double> & diagA, std::vector<Color> & residual_color_color, std::vector<Color> & diagA_color) const;
    void compute_Ap(std::vector<double> & Ap, std::vector<double> const& p, std::vector<Color> & Ap_color, std::vector<Color> & p_color) const;

    template <typename T>
    static inline T dot_ab_over_dot_cd_(std::vector<T> const& a, std::vector<T> const& b,
                                      std::vector<T> const& c, std::vector<T> const& d)
    {
        return std::inner_product(a.begin(), a.end(), b.begin(), T())/std::inner_product(c.begin(), c.end(), d.begin(), T());
    }

public:
    ColorSmoothSignedDistance(Octree & octree, BBox const& bbox, OctreeDataType const& data, std::vector<PointType> const& points);

    void set_config(Settings const& config);

    size_t relax(bool fast = false);
    void transfer_solution_to_cells(void);
};

#include "ColorSmoothSignedDistance.inl"

#endif // __COLORSMOOTHSIGNEDDISTANCE_HPP__
