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

#include <cmath>
#include <cassert>
#include <cstdio>
#include <functional>
#include <numeric>

#ifdef NO_DATA
#undef NO_DATA
#endif //NO_DATA

template <typename Real> template <typename PointType> void SSDCellData<Real>::set_point_color(PointType & p) const
{
}

template <typename Octree, typename OctreeDataType, typename PointType>
const double SmoothSignedDistance<Octree,OctreeDataType,PointType>::grad[3][8] =
{
    { -0.25,-0.25,-0.25,-0.25, 0.25, 0.25, 0.25, 0.25 },
    { -0.25,-0.25, 0.25, 0.25,-0.25,-0.25, 0.25, 0.25 },
    { -0.25, 0.25,-0.25, 0.25,-0.25, 0.25,-0.25, 0.25 }
};

template <typename Octree, typename OctreeDataType, typename PointType>
SmoothSignedDistance<Octree,OctreeDataType,PointType>::SmoothSignedDistance(Octree & octree, BBox const& bbox, OctreeDataType const& data, std::vector<PointType> const& points) :
    _octree(octree),
    _bbox(bbox),
    _data(data),
    _points(points),
    _lambda1(1.0),
    _lambda2(1.0),
    _lambda3(1.0), 
    _tol(1e-8),
    _miniter(1),
    _maxiter(300),
    _normalizer1(0.0),
    _normalizer2(0.0),
    _normalizer3(0.0)
{ 
}

template <typename Octree, typename OctreeDataType, typename PointType>
void SmoothSignedDistance<Octree,OctreeDataType,PointType>::set_config(Settings const& config)
{
  double sumofweights = config.weight0+config.weight1+config.weight2;
  _lambda1 = config.weight0/sumofweights;
  _lambda2 = config.weight1/sumofweights;
  _lambda3 = config.weight2/sumofweights;
  _tol = config.solverTolerance;

  _miniter = config.minIterations;
  _maxiter = config.maxIterations;
}

template <typename Octree, typename OctreeDataType, typename PointType>
size_t SmoothSignedDistance<Octree,OctreeDataType,PointType>::relax(bool fast)
{
    size_t miniter   = _miniter;
    size_t maxiter   = _maxiter;
    double tol       = _tol;

    if (fast)
    {   //finish early
        miniter     = 1;
        maxiter     = 50;
        tol         = 10.0*tol;
    }

    // allocate nVx1 zero vectors 
    size_t nV = _octree.getNumberOfVertices();
    std::vector<double> r(nV), diagA(nV);

    accumulate_normalizers();
    accumulate(r, diagA);

    std::vector<double> z(nV), p(nV), rn(nV), zn(nV);

    for (size_t i=0; i<nV; ++i)
    {                               //: M = diag(A);
        if(diagA[i]<1.0e-8) 
        {   //avoid division-by-zero
            diagA[i] = 1.0e-8;
        }
        z[i] = r[i]/diagA[i];       //: z = M\r;
        p[i] = z[i];                //: p = z;
    }

    size_t niter = 0;
    double gamma = tol+1.0;
    std::vector<double> Ap(nV);
    while ((gamma>tol || niter<miniter) && niter<maxiter)
    {
        Ap.assign(nV, 0.0);
        compute_Ap(Ap, p);

        double alpha = dot_ab_over_dot_cd_(z, r, p, Ap);  //: alpha = (z'*r)/(p'*A*p);

        typename Octree::vertex_iterator vertex_iter = _octree.vertex_begin();
        for (size_t i=0; i<nV; ++i, ++vertex_iter)
        {
            //*(_bundle.cache._vertices[i]) += static_cast<Octree::VertexData>(alpha*p[i]); //: f += alpha*p;
            (*vertex_iter).second->f += static_cast<typename Octree::VertexData::Real>(alpha*p[i]); //: f += alpha*p;
            rn[i] = r[i] - alpha*Ap[i];      //: rn = r - alpha*A*p;
            zn[i] = rn[i]/diagA[i];          //: zn = M\rn;
        }

        double beta = dot_ab_over_dot_cd_(zn, rn, z, r); //: beta = (zn'*rn)/(z'*r);
        
        gamma = 0.0;
        for (size_t i=0; i<nV; ++i)
        {
            r[i] = rn[i];                 //: r = rn;
            z[i] = zn[i];                 //: z = zn;
            p[i] = z[i] + beta*p[i];      //: p = z + beta*p;
            if (std::abs(r[i])>gamma) { gamma = std::abs(r[i]); }
        }

        ++niter;
    }
    
    return niter;
}

template <typename Octree, typename OctreeDataType, typename PointType>
void SmoothSignedDistance<Octree,OctreeDataType,PointType>::accumulate_normalizers(void)
{
    /* all points have weight = 1.0, there's no need to accumulate unless this changes */
    double point_count = static_cast<double>(_points.size());
    _normalizer1 = point_count;
    _normalizer2 = point_count;
    _normalizer3 = 0.0;

    for (typename Octree::cell_iterator iter=_octree.cell_begin(); iter!=_octree.cell_end(); ++iter)
    {
        typename Octree::CellKey cellA = (*iter).first;
        size_t cell_a[4];
        _octree.getCellCoords(cellA, cell_a);
        typename Octree::Vertex vertices_a[8];
        _octree.getVertices(cellA, vertices_a);
        typename Octree::CellKey neighbors[6];
        _octree.getFaceNeighborKeys(cellA, neighbors);
        const double da = _octree.getCellWidth(cell_a[0]);

        for(size_t edge=0; edge<6; ++edge)
        {
            typename Octree::CellKey cellB = neighbors[edge];
            if (!_octree.isValidCellKey(cellB))
            {
                continue;
            }

            size_t cell_b[4];
            _octree.getCellCoords(cellB, cell_b);
            typename Octree::Vertex vertices_b[8];
            _octree.getVertices(cellB, vertices_b);
            const double db = _octree.getCellWidth(cell_b[0]);

            // determine the dual edge length (distance between primal cell centers)
            double dm_vec[3] = {(cell_a[1]+0.5)*da - (cell_b[1]+0.5)*db, (cell_a[2]+0.5)*da - (cell_b[2]+0.5)*db, (cell_a[3]+0.5)*da - (cell_b[3]+0.5)*db};
            double dm = sqrt(dm_vec[0]*dm_vec[0]+dm_vec[1]*dm_vec[1]+dm_vec[2]*dm_vec[2]);

            _normalizer3 += 1.0/dm;
        }
    }

    // since multiplication is faster than division, keep the reciprocals
    _normalizer1 = _lambda1/_normalizer1;
    _normalizer2 = _lambda2/_normalizer2;
    _normalizer3 = _lambda3/_normalizer3;
}

template <typename Octree, typename OctreeDataType, typename PointType>
void SmoothSignedDistance<Octree,OctreeDataType,PointType>::accumulate(std::vector<double> & residual, std::vector<double> & diagA) const
{
    double bbox_side[3] = {_bbox.getSide(0), _bbox.getSide(1), _bbox.getSide(2)};

    // go over the cells and compute A'*A*f, A'*b, and diag(A'*A)
    std::vector<typename Octree::VertexKey> is_used(residual.size(), _octree.INVALID_VERTEX_KEY);

    for (typename Octree::cell_iterator iter=_octree.cell_begin(); iter!=_octree.cell_end(); ++iter)
    {
        typename Octree::CellKey cellA = (*iter).first;
        size_t cell_a[4];
        _octree.getCellCoords(cellA, cell_a);
        typename Octree::Vertex vertices_a[8];
        _octree.getVertices(cellA, vertices_a);
        typename Octree::VertexKey vkey_a[8] = {vertices_a[0].first, vertices_a[1].first, vertices_a[2].first, vertices_a[3].first, 
                                                vertices_a[4].first, vertices_a[5].first, vertices_a[6].first, vertices_a[7].first};
        typename Octree::CellKey neighbors[6];
        _octree.getFaceNeighborKeys(cellA, neighbors);
        typename Octree::CellData * data_a = (*iter).second;
        const double da = _octree.getCellWidth(cell_a[0]);

        double fa[8];
        for (size_t i=0; i<8; ++i)
        {
            fa[i] = vertices_a[i].second->f;
        }

        if (data_a->first_point!=Octree::CellData::NO_DATA)
        {
            // get bbox of cell alpha
            double v0[3] = {_bbox.getMin(0) + cell_a[1]*da*_bbox.getSide(0),
                            _bbox.getMin(1) + cell_a[2]*da*_bbox.getSide(1),
                            _bbox.getMin(2) + cell_a[3]*da*_bbox.getSide(2)};
            double scale[3] = {1.0/(da*bbox_side[0]), 1.0/(da*bbox_side[1]), 1.0/(da*bbox_side[2])};

            // go over the points enclosed in cell alpha, and accumulate their contribution
            for (size_t iP=data_a->first_point; iP!=Octree::CellData::NO_DATA; iP=_data.getNextPoint(iP))
            {
                // get point pi in cell coordinates
                PointType const& pt = _points[iP];
                const double px = std::min(1.0,std::max(0.0,(pt.x-v0[0])*scale[0]));
                const double py = std::min(1.0,std::max(0.0,(pt.y-v0[1])*scale[1]));
                const double pz = std::min(1.0,std::max(0.0,(pt.z-v0[2])*scale[2]));

                const double one_minus_px = 1.0-px;
                const double one_minus_py = 1.0-py;
                const double one_minus_pz = 1.0-pz;

                // trilinear
                const double T[8] =  { one_minus_px*one_minus_py*one_minus_pz, one_minus_px*one_minus_py*pz, one_minus_px*py*one_minus_pz, 
                                       one_minus_px*py*pz, px*one_minus_py*one_minus_pz, px*one_minus_py*pz, px*py*one_minus_pz, px*py*pz};
                // accumulate dT/dx
                const double Tx[8] = {-one_minus_py*one_minus_pz,-one_minus_py*pz,-py*one_minus_pz,-py*pz, 
                                       one_minus_py*one_minus_pz, one_minus_py*pz, py*one_minus_pz, py*pz};
                // accumulate dT/dy
                const double Ty[8] = {-one_minus_px*one_minus_pz,-one_minus_px*pz, one_minus_px*one_minus_pz, one_minus_px*pz,
                                      -px*one_minus_pz,-px*pz, px*one_minus_pz, px*pz};
                // accumulate dT/dz
                const double Tz[8] = {-one_minus_px*one_minus_py, one_minus_px*one_minus_py,-one_minus_px*py, one_minus_px*py,
                                      -px*one_minus_py, px*one_minus_py,-px*py, px*py};

                double TF = 0.0, TxF = 0.0, TyF = 0.0, TzF = 0.0;

                for (size_t i=0; i<8; ++i)
                {
                    TF  += T[i]*fa[i];   // f(pi) = T(pi)' * F                    [A0]
                    TxF += Tx[i]*fa[i];  // df(pi)/dx = Tx(pi)' * F -> grad f(pi) [A1]
                    TyF += Ty[i]*fa[i];  // df(pi)/dy = Ty(pi)' * F
                    TzF += Tz[i]*fa[i];  // df(pi)/dz = Tz(pi)' * F
                }

                for (size_t i=0; i<8; ++i)
                {  
                    size_t index = vkey_a[i];
                    residual[index] += _normalizer2*da*(pt.nx*Tx[i] + pt.ny*Ty[i] + pt.nz*Tz[i]) // grad f(pi)' * ni
                                    - (_normalizer1*TF*T[i] + _normalizer2*(TxF*Tx[i] +  TyF*Ty[i] +  TzF*Tz[i]));
                              // [T(pi)' * F] * T(pi,vk)  +  [Tx(pi)' * F] * Tx(pi,vk) + [Ty(pi)' * F] * Ty(pi,vk) + [Ty(pi)' * F] * Ty(pi,vk)

                    diagA[index] += _normalizer1*T[i]*T[i] + _normalizer2*(Tx[i]*Tx[i] + Ty[i]*Ty[i] + Tz[i]*Tz[i]);
                                 //  // T(pi,vk)^2         +               Tx(pi,vk)^2 + Ty(pi,vk)^2 + Tz(pi,vk)^2
                }
            }   //for points in cell
        }//if cell occupied

        for(size_t edge=0; edge<6; ++edge)
        {
            typename Octree::CellKey cellB = neighbors[edge];
            if (!_octree.isValidCellKey(cellB))
            {
                continue;
            }

            size_t cell_b[4];
            _octree.getCellCoords(cellB, cell_b);
            typename Octree::Vertex vertices_b[8];
            _octree.getVertices(cellB, vertices_b);
            typename Octree::VertexKey vkey_b[8] = {vertices_b[0].first, vertices_b[1].first, vertices_b[2].first, vertices_b[3].first, 
                                                    vertices_b[4].first, vertices_b[5].first, vertices_b[6].first, vertices_b[7].first};
            const double db = _octree.getCellWidth(cell_b[0]);

            // determine the dual edge length (distance between primal cell centers)
            double dm_vec[3] = {(cell_a[1]+0.5)*da - (cell_b[1]+0.5)*db, (cell_a[2]+0.5)*da - (cell_b[2]+0.5)*db, (cell_a[3]+0.5)*da - (cell_b[3]+0.5)*db};
            double dm = sqrt(dm_vec[0]*dm_vec[0]+dm_vec[1]*dm_vec[1]+dm_vec[2]*dm_vec[2]);

            //double mindab = min(da, db);
            //mindab = (mindab/da)*(mindab/db)*(1/dm);
            double mindab = 1.0/dm;

            double geom_mean = sqrt(da*db);
            double dag = geom_mean/da;
            double dbg = geom_mean/db;

            double GxF = 0.0, GyF = 0.0, GzF = 0.0;
            for (size_t i=0; i<8; ++i)
            {
                size_t i_index = vkey_a[i];
                size_t j_index = vkey_b[i];

                is_used[i_index] = _octree.INVALID_VERTEX_KEY;
                is_used[j_index] = _octree.INVALID_VERTEX_KEY;

                double fb = vertices_b[i].second->f;
                GxF += fa[i]*grad[0][i]*dag - fb*grad[0][i]*dbg; //fx2-fx1 = Gx' * F
                GyF += fa[i]*grad[1][i]*dag - fb*grad[1][i]*dbg; //fy2-fy1 = Gy' * F
                GzF += fa[i]*grad[2][i]*dag - fb*grad[2][i]*dbg; //fz2-fz1 = Gz' * F
            }

            double w = _normalizer3*mindab;
            double daw = dag*w;
            double dbw = dbg*w;

            for (size_t i=0; i<8; ++i)
            {
                size_t i_index = vkey_a[i];
                size_t j_index = vkey_b[i];

                residual[i_index] -= daw*(GxF*grad[0][i]+GyF*grad[1][i]+GzF*grad[2][i]);
                                 //  Gx(a,i) * (Gx(a)'*F) + Gy(a,i) * (Gy(a)'*F) + Gz(a,i) * (Gz(a)'*F) 
                residual[j_index] += dbw*(GxF*grad[0][i]+GyF*grad[1][i]+GzF*grad[2][i]);
                                 // -Gx(b,i) * (Gx(b)'*F) - Gy(b,i) * (Gy(b)'*F) - Gz(b,i) * (Gz(b)'*F)

                is_used[i_index] = i;
            }

            double cda2w = 0.1875*dag*dag*w; // 3*0.25*0.25*da*da*mindab*normalizer3_
            double cdb2w = 0.1875*dbg*dbg*w;
            double twodadabw = 2*dag*dbg*w;
            for (size_t i=0; i<8; ++i)
            {
                size_t i_index = vkey_a[i];
                size_t j_index = vkey_b[i];
                diagA[i_index] += cda2w;
                diagA[j_index] += cdb2w;
                size_t ii = is_used[j_index];
                if (ii<_octree.INVALID_VERTEX_KEY)
                {
                    diagA[j_index] -= twodadabw*(grad[0][i]*grad[0][ii]+grad[1][i]*grad[1][ii]+grad[2][i]*grad[2][ii]);
                }
            }
        }//for neighbors
    }//for cells
}

template <typename Octree, typename OctreeDataType, typename PointType>
void SmoothSignedDistance<Octree,OctreeDataType,PointType>::compute_Ap(std::vector<double> & Ap, std::vector<double> const& p) const
{
    double bbox_side[3] = {_bbox.getSide(0), _bbox.getSide(1), _bbox.getSide(2)};

    // go over the cells and compute A'*A*f, A'*b, and diag(A'*A)

    for (typename Octree::cell_iterator iter=_octree.cell_begin(); iter!=_octree.cell_end(); ++iter)
    {
        typename Octree::CellKey cellA = (*iter).first;
        size_t cell_a[4];
        _octree.getCellCoords(cellA, cell_a);
        typename Octree::Vertex vertices_a[8];
        _octree.getVertices(cellA, vertices_a);
        typename Octree::VertexKey vkey_a[8] = {vertices_a[0].first, vertices_a[1].first, vertices_a[2].first, vertices_a[3].first, 
                                                vertices_a[4].first, vertices_a[5].first, vertices_a[6].first, vertices_a[7].first};
        typename Octree::CellKey neighbors[6];
        _octree.getFaceNeighborKeys(cellA, neighbors);
        typename Octree::CellData * data_a = (*iter).second;
        const double da = _octree.getCellWidth(cell_a[0]);

        double pa[8];
        for (size_t i = 0; i<8; ++i)
        {
            pa[i] = p[vkey_a[i]];
        }

        if (data_a->first_point!=Octree::CellData::NO_DATA)
        {
            // get bbox of cell alpha 
            double v0[3] = {_bbox.getMin(0) + cell_a[1]*da*_bbox.getSide(0),
                            _bbox.getMin(1) + cell_a[2]*da*_bbox.getSide(1),
                            _bbox.getMin(2) + cell_a[3]*da*_bbox.getSide(2)};
            double scale[3] = {1.0/(da*bbox_side[0]), 1.0/(da*bbox_side[1]), 1.0/(da*bbox_side[2])};

            // go over the points enclosed in cell alpha, and accumulate their contribution
            for (size_t iP=data_a->first_point; iP!=Octree::CellData::NO_DATA; iP=_data.getNextPoint(iP))
            {
                // get point pi in cell coordiates
                PointType const& pt = _points[iP];
                const double px = std::min(1.0,std::max(0.0,(pt.x-v0[0])*scale[0]));
                const double py = std::min(1.0,std::max(0.0,(pt.y-v0[1])*scale[1]));
                const double pz = std::min(1.0,std::max(0.0,(pt.z-v0[2])*scale[2]));

                const double one_minus_px = 1.0-px;
                const double one_minus_py = 1.0-py;
                const double one_minus_pz = 1.0-pz;

                // trilinear
                const double T[8] =  { one_minus_px*one_minus_py*one_minus_pz, one_minus_px*one_minus_py*pz, one_minus_px*py*one_minus_pz, 
                                       one_minus_px*py*pz, px*one_minus_py*one_minus_pz, px*one_minus_py*pz, px*py*one_minus_pz, px*py*pz};
                // accumulate dT/dx
                const double Tx[8] = {-one_minus_py*one_minus_pz,-one_minus_py*pz,-py*one_minus_pz,-py*pz, 
                                       one_minus_py*one_minus_pz, one_minus_py*pz, py*one_minus_pz, py*pz};
                // accumulate dT/dy
                const double Ty[8] = {-one_minus_px*one_minus_pz,-one_minus_px*pz, one_minus_px*one_minus_pz, one_minus_px*pz,
                                      -px*one_minus_pz,-px*pz, px*one_minus_pz, px*pz};
                // accumulate dT/dz
                const double Tz[8] = {-one_minus_px*one_minus_py, one_minus_px*one_minus_py,-one_minus_px*py, one_minus_px*py,
                                      -px*one_minus_py, px*one_minus_py,-px*py, px*py};

                double TF = 0.0, TxF = 0.0, TyF = 0.0, TzF = 0.0;

                for (size_t i=0; i<8; ++i)
                {
                    TF  += T[i]*pa[i];   // f(pi) = T(pi)' * F                    [A0]
                    TxF += Tx[i]*pa[i];  // df(pi)/dx = Tx(pi)' * F -> grad f(pi) [A1]
                    TyF += Ty[i]*pa[i];  // df(pi)/dy = Ty(pi)' * F
                    TzF += Tz[i]*pa[i];  // df(pi)/dz = Tz(pi)' * F
                }

                for (size_t i=0; i<8; ++i)
                {  
                    size_t index = vkey_a[i];
                    Ap[index] += _normalizer1*TF*T[i] + _normalizer2*(TxF*Tx[i] +  TyF*Ty[i] +  TzF*Tz[i]);
                              // [T(pi)' * F] * T(pi,vk)  +  [Tx(pi)' * F] * Tx(pi,vk) + [Ty(pi)' * F] * Ty(pi,vk) + [Ty(pi)' * F] * Ty(pi,vk)
                }
            }   //for points in cell
        }//if cell occupied

        for(size_t edge=0; edge<6; ++edge)
        {
            typename Octree::CellKey cellB = neighbors[edge];
            if (!_octree.isValidCellKey(cellB))
            {
                continue;
            }

            size_t cell_b[4];
            _octree.getCellCoords(cellB, cell_b);
            typename Octree::Vertex vertices_b[8];
            _octree.getVertices(cellB, vertices_b);
            typename Octree::VertexKey vkey_b[8] = {vertices_b[0].first, vertices_b[1].first, vertices_b[2].first, vertices_b[3].first, 
                                                    vertices_b[4].first, vertices_b[5].first, vertices_b[6].first, vertices_b[7].first};
            const double db = _octree.getCellWidth(cell_b[0]);

            // determine the dual edge length (distance between primal cell centers)
            double dm_vec[3] = {(cell_a[1]+0.5)*da - (cell_b[1]+0.5)*db, (cell_a[2]+0.5)*da - (cell_b[2]+0.5)*db, (cell_a[3]+0.5)*da - (cell_b[3]+0.5)*db};
            double dm = sqrt(dm_vec[0]*dm_vec[0]+dm_vec[1]*dm_vec[1]+dm_vec[2]*dm_vec[2]);

            //double mindab = min(da, db);
            //mindab = (mindab/da)*(mindab/db)*(1/dm);
            double mindab = 1.0/dm;

            double geom_mean = sqrt(da*db);
            double dag = geom_mean/da;
            double dbg = geom_mean/db;

            double GxF = 0.0, GyF = 0.0, GzF = 0.0;
            for (size_t i=0; i<8; ++i)
            {
                double pb = p[vkey_b[i]];
                GxF += pa[i]*grad[0][i]*dag - pb*grad[0][i]*dbg; //fx2-fx1 = Gx' * F
                GyF += pa[i]*grad[1][i]*dag - pb*grad[1][i]*dbg; //fy2-fy1 = Gy' * F
                GzF += pa[i]*grad[2][i]*dag - pb*grad[2][i]*dbg; //fz2-fz1 = Gz' * F
            }

            double w = _normalizer3*mindab;
            double daw = dag*w;
            double dbw = dbg*w;

            for (size_t i=0; i<8; ++i)
            {
                size_t i_index = vkey_a[i];
                size_t j_index = vkey_b[i];

                Ap[i_index] += daw*(GxF*grad[0][i]+GyF*grad[1][i]+GzF*grad[2][i]);
                              //  Gx(a,i) * (Gx(a)'*F) + Gy(a,i) * (Gy(a)'*F) + Gz(a,i) * (Gz(a)'*F)
                Ap[j_index] -= dbw*(GxF*grad[0][i]+GyF*grad[1][i]+GzF*grad[2][i]);
                              // -Gx(b,i) * (Gx(b)'*F) - Gy(b,i) * (Gy(b)'*F) - Gz(b,i) * (Gz(b)'*F)
            }

        }//for neighbors
    }//for cells
}

template <typename Octree, typename OctreeDataType, typename PointType>
void SmoothSignedDistance<Octree,OctreeDataType,PointType>::transfer_solution_to_cells(void)
{
    for (typename Octree::cell_iterator iter=_octree.cell_begin(); iter!=_octree.cell_end(); ++iter)
    {
        typename Octree::CellKey cellA = (*iter).first;
        typename Octree::Vertex vertices_a[8];
        _octree.getVertices(cellA, vertices_a);
        typename Octree::CellData * data_a = (*iter).second;

        data_a->f = 0.125f * ( vertices_a[0].second->f + vertices_a[1].second->f + vertices_a[2].second->f + vertices_a[3].second->f
                             + vertices_a[4].second->f + vertices_a[5].second->f + vertices_a[6].second->f + vertices_a[7].second->f);
    }
 }
