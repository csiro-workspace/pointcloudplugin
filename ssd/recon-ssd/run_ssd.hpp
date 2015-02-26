/*
Copyright (c) 2011, Fatih Calakli and Gabriel Taubin
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

#ifndef __RUN_SSH_HPP__
#define __RUN_SSH_HPP__

#include <string.h>

#include "Settings.hpp"
#include "octree/OctreeBundle.hpp"
#include "isosurface/DualMarchingCubes.hpp"

#include "HRTimer.hpp"

#define SSD_VER_MAJOR 3
#define SSD_VER_MINOR 0

template <typename SSDType> void run_ssd(Settings const& D, OctreeBundle<SSDType> & bundle, std::vector<typename SSDType::OutPointType> & vertices, std::vector<std::vector<size_t> > & faces, volatile double * progress = NULL)
{
    //////////////////////////////////////////////////////////////////////
    // initialize octree

#ifdef _MSC_VER
    //time measurements
    HRTimer timer;
    timer.start();
#endif

    if (progress) {*progress = 0.0;}

    if(D.debug) 
    {
        fprintf(stdout,"  constructing initial octree {\n");
        fflush(stdout);
    }

    bundle.init();
    bundle.bbox.expand(D.bboxExpansionFactor);

    if(D.debug) 
    {
        fprintf(stdout,"    center = (%8.4lf %8.4lf %8.4lf)\n", bundle.bbox.getCenter(0), bundle.bbox.getCenter(1), bundle.bbox.getCenter(2));
        fprintf(stdout,"    side   = (%8.4lf %8.4lf %8.4lf)\n", bundle.bbox.getSide(0), bundle.bbox.getSide(1), bundle.bbox.getSide(2));
        fprintf(stdout,"  }\n");
        fprintf(stdout,"\n");
        fflush(stdout);
    }

    bundle.octree_refine(D.octreeLevelStart-1);
  
    //////////////////////////////////////////////////////////////////////
    // initialize smooth signed distance
    if(D.debug) {
        fprintf(stdout,"  estimating smooth signed distance function {\n\n");
        fprintf(stdout,"    initializing \n\n");
        fflush(stdout);
    }

    ////////////////////////////////////////////////////////////////////////
    //// refine smooth signed distance

    if (D.debug) 
    {
        fprintf(stdout,"    refining \n\n");
        fflush(stdout);
    }

    bool need_refine = true;
    while (need_refine)
    {
        if (progress) {*progress += 100.0/(D.octreeLevels+1);}
        size_t splits = 0;
        size_t L = bundle.octree.getLevels();
        size_t nVnew = 0; 
        if (L<D.octreeLevels)
        {
            nVnew = bundle.octree.getNumberOfVertices();
            splits = bundle.octree_refine(L+1);
            nVnew = bundle.octree.getNumberOfVertices()-nVnew;
        }
        need_refine = (splits>0 && bundle.octree.getLevels()<D.octreeLevels);

        if (D.debug) 
        {
            fprintf(stdout,"    depth      = %d\n", bundle.octree.getLevels());
            fprintf(stdout,"    nCells     = %d\n", bundle.octree.getNumberOfCells());
            fprintf(stdout,"    nVertices  = %d\n", bundle.octree.getNumberOfVertices());
            fprintf(stdout,"    nVnew      = %d\n", nVnew);
            fflush(stdout);
        }

        if (bundle.octree.getLevels()==D.octreeLevels || !need_refine)
        {   //last iteration: balance
#ifdef _MSC_VER
            HRTimer timer0; timer0.start();
#endif
            if (D.debug) {
                fprintf(stdout,"\n    { balancing octree");
            }        
            
            nVnew = bundle.octree.getNumberOfVertices();
            bundle.octree_balance(1);
            nVnew = bundle.octree.getNumberOfVertices()-nVnew;

            if (D.debug) 
            {
#ifdef _MSC_VER
                timer0.stop();
                fprintf(stdout," [%lf seconds]", timer0.elapsed());
#endif
                fprintf(stdout,"\n      depth      = %d\n", bundle.octree.getLevels());
                fprintf(stdout,"      nCells     = %d\n", bundle.octree.getNumberOfCells());
                fprintf(stdout,"      nVertices  = %d\n", bundle.octree.getNumberOfVertices());
                fprintf(stdout,"      nVnew      = %d\n", nVnew);

                fprintf(stdout,"    }\n");
                fflush(stdout);
            }
        }

        bool fast = (D.fast && need_refine);
        bundle.cache.build(bundle.octree);
        size_t niter = bundle.ssd.relax(fast);

        if (D.debug) {
            fprintf(stdout,"    nIters     = %d\n\n", niter);
            fflush(stdout);
        }

        bool debug_octree = false;
        //debug_octree = true;
        if (D.debug && debug_octree) {
            bundle.octree.print_stats();     //this might be time consuming
            fprintf(stdout, "\n");
        }

        if (!D.debug)
        {
          fprintf(stdout, " Done: %d%%\r", static_cast<int>(bundle.octree.getLevels()*100.0/(D.octreeLevels+1)));
          fflush(stdout);
        }

    } //while

    if(D.debug) {
        // } estimating smooth signed distance function
        fprintf(stdout,"  }\n\n");
        fflush(stdout);
    }

    //////////////////////////////////////////////////////////////////////
    // extract isosurface and save

#ifdef _MSC_VER
    HRTimer timer0; timer0.start();
#endif
    if(D.debug) {
        fprintf(stdout,"  extracting dual isosurface {\n");
        fflush(stdout);
    }

    bundle.ssd.transfer_solution_to_cells();
    DualMarchingCubes::getIsoSurface(bundle.octree, bundle.bbox, D.isoLevel, vertices, faces, true);//SRM

    //convert polygons to triangles
    std::vector<std::vector<size_t> > triangles;

    if (!D.polygons)
    {
        DualMarchingCubes::PolygonToTriangleMesh(vertices, faces, triangles);
        faces.swap(triangles);
    }
    if (D.manifold)
    {
        DualMarchingCubes::PolygonToManifoldTriangleMesh(vertices, faces, triangles);
        faces.swap(triangles);
    }


    if(D.debug) 
    {
        fprintf(stdout,"    %d vertices\n", vertices.size());
        fprintf(stdout,"    %d faces\n"   , faces.size());
#ifdef _MSC_VER
        timer0.stop();
        fprintf(stdout,"    %lf seconds\n"   , timer0.elapsed());
#endif
        fprintf(stdout,"  }\n\n");
        fflush(stdout);
    }

    if (progress) {*progress = 100.0;}

#ifdef _MSC_VER
    timer.stop();
    if(D.debug) {
        fprintf(stdout,"  total time   = %lf\n\n", timer.elapsed());
    }
#endif

    if (!D.debug)
    {
      fprintf(stdout, "           \r");
      fflush(stdout);
    }
}

#endif //__RUN_SSH_HPP__