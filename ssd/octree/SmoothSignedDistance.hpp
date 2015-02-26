/*
Copyright (c) 2011, Fatih Calakli
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

#ifndef __SMOOTHSIGNEDDISTANCE_HPP__
#define __SMOOTHSIGNEDDISTANCE_HPP__

#include <vector>
#include <numeric>

template <typename Octree, typename OctreeDataType, typename PointType>
class SmoothSignedDistance 
{
    Octree & _octree;
    BBox const& _bbox;
    OctreeDataType const& _data;
    std::vector<PointType> const& _points;

    double _lambda1;
    double _lambda2;
    double _lambda3;
    double _tol;
    size_t _miniter;
    size_t _maxiter;
    double _normalizer1;
    double _normalizer2;
    double _normalizer3;

    static const double grad[3][8];

    void accumulate_normalizers(void);
    void accumulate(std::vector<double> & residual, std::vector<double> & diagA) const;
    void compute_Ap(std::vector<double> & Ap, std::vector<double> const& p) const;

    static inline double dot_ab_over_dot_cd_(std::vector<double> const& a, std::vector<double> const& b,
                                      std::vector<double> const& c, std::vector<double> const& d)
    {
        return std::inner_product(a.begin(), a.end(), b.begin(), 0.0)/std::inner_product(c.begin(), c.end(), d.begin(), 0.0);
    }

public:
    SmoothSignedDistance(Octree & octree, BBox const& bbox, OctreeDataType const& data, std::vector<PointType> const& points);

    void set_config(Settings const& config);

    size_t relax(bool fast = false);
    void transfer_solution_to_cells(void);
};

#include "SmoothSignedDistance.inl"

#endif // __SMOOTHSIGNEDDISTANCE_HPP__
