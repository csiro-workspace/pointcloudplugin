/*============================================================================

  Copyright 2015 by:

    Commonwealth Scientific and Industrial Research Organisation (CSIRO)
    
    This file is licensed by CSIRO under the copy of the CSIRO Open Source
    Software License Agreement (variation of the BSD / MIT License) included
    with the file.

    As a condition of this license, you agree that where you make any 
    adaptations, modifications, further developments, or additional features 
    available to CSIRO or the public in connection with your access to the 
    Software, you do so on the terms of the BSD 3-Clause License template,
    a copy available at: http://opensource.org/licenses/BSD-3-Clause

    For further information, contact the CSIRO Workspace Team:
    workspace@csiro.au

  This copyright notice must be included with all copies of the source code.

============================================================================*/

/*

Copyright (c) 2015, Stuart Mead - Risk Frontiers, Macquarie University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      
    * Neither the name of the copyright holder nor the names of its contributors
	  may be used to endorse or promote products derived from this software without
	  specific prior written permission.

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

  For further information, contact:
    Stuart Mead
	Risk Frontiers
    Dept. of Environmental Sciences
    Macquarie University
    North Ryde NSW 2109
	
*/

#include <cassert>
#include <iostream>

#include <QString>

#include "Workspace/Application/LanguageUtils/streamqstring.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"
#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterface.h"
#include "Mesh/DataStructures/MeshModelInterface/meshelementsinterface.h"
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"

#include "pointcloudplugin.h"
#include "movingleastsquares.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/mls_omp.h>

namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{

    /**
     * \internal
     */
    class MovingLeastSquaresImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(MovingLeastSquaresImpl)

    public:
        MovingLeastSquares&  op_;

        // Data objects
        DataExecution::TypedObject< MeshModelInterface >  dataMesh_;
        DataExecution::TypedObject< bool >                dataOMP_;
        DataExecution::TypedObject< MeshModelInterface >  dataOutputMesh_;
        DataExecution::TypedObject< double >              searchRadius_;
        DataExecution::TypedObject< int >                 polyOrder_;
        DataExecution::TypedObject< bool >                polyFit_;


        // Inputs and outputs
        DataExecution::InputScalar inputMesh_;
        DataExecution::InputScalar inputOMP_;
        DataExecution::InputScalar inputSearchRadius_;
        DataExecution::InputScalar inputpolyOrder_;
        DataExecution::InputScalar inputpolyFit_;
        DataExecution::Output      outputMesh_;


        MovingLeastSquaresImpl(MovingLeastSquares& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    MovingLeastSquaresImpl::MovingLeastSquaresImpl(MovingLeastSquares& op) :
        op_(op),
        dataMesh_(),
        dataOMP_(false),
        dataOutputMesh_(),
        searchRadius_(0.03),
        polyOrder_(2),
        polyFit_(true),
        inputMesh_("Points", dataMesh_, op_),
        inputOMP_("Use OpenMP", dataOMP_, op_),
        inputSearchRadius_("Search Radius", searchRadius_, op_),
        inputpolyOrder_("Polynomial Order", polyOrder_, op_),
        inputpolyFit_("Polynomial Fitting", polyFit_, op_),
        outputMesh_("Mesh Model", dataOutputMesh_, op_)
    {
        inputSearchRadius_.setDescription("Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting");
        inputOMP_.setDescription("Enables OpenMP based computation, threads will be determined on the OMP_NUM_THREADS environment variable");
        inputpolyOrder_.setDescription("Order of the polynomial to be fit, if polynomial fitting is true");
        inputpolyFit_.setDescription("Approximate the surface and normal using a polynomial, if not, tangent estimation is used");
    }


    /**
     *
     */
    bool MovingLeastSquaresImpl::execute()
    {
        MeshModelInterface& mesh = *dataMesh_;
        MeshModelInterface& outputMesh = *dataOutputMesh_;

        MeshNodesInterface& nodes = mesh.getNodes();
       /* if (!nodes.hasState("RGBA"))
       {
            std::cout << QString("ERROR: Nodes do not have RGBA state") + "\n";
            return false;
       }

        NodeStateHandle inputRgba = nodes.getStateHandle("RGBA");*/
        outputMesh.clear();//Always clear outputmesh
        MeshNodesInterface& outputNodes = outputMesh.getNodes();

        //Kinect stuff
        const NodeStateHandle* pixIdStateIn = 0;
        const NodeStateHandle* textureSStateOut = 0;
        const NodeStateHandle* textureTStateOut = 0;
        if (nodes.hasState("pixId"))
        {
            pixIdStateIn = &nodes.getStateHandle("pixId");
            textureSStateOut = &outputNodes.addState<double>("textureS", 0.0);
            textureTStateOut = &outputNodes.addState<double>("textureT", 0.0);
        }

    
        //Add states for normal and curvature
        const NodeStateHandle& normalStateOut = outputNodes.addState("normal", Vector3d(0,0,-1));
        const NodeStateHandle& curvatureStateOut = outputNodes.addState<double>("curvature",0.0);

        //Finish if input mesh is empty
        if(nodes.size() == 0)
        {
            std::cout << QString("WARNING: Mesh has no nodes, returning successfully") + "\n";
            return true;
        }
    
        //For each node add point to point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba (new pcl::PointCloud<pcl::PointXYZRGBA>);
        MeshNodesInterface::const_iterator nIter = nodes.begin();
        MeshNodesInterface::const_iterator end = nodes.end();
        for(; nIter != end; ++nIter)
        {
           int rgba = 0;
           Vector3d v = nodes.getPosition(*nIter);
           //nodes.getState(*nIter,inputRgba,rgba);
           pcl::PointXYZ p;
           p.x = v.x;
           p.y = v.y;
           p.z = v.z;
           //p.rgb = rgba;
           cloud->push_back(pcl::PointXYZ(p));
        }
        std::cout << QString("Point cloud generated, has %1 points").arg(cloud->size()) + "\n";

        // Create a KD-Tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        // Output has the PointNormal type in order to store the normals from mls
        pcl::PointCloud<pcl::PointNormal> mls_cloud;

        if (*dataOMP_)
        {
            // Init object (second point type is for the normals, even if unused)
            pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;
            char * ompThreads;
            ompThreads = getenv("OMP_NUM_THREADS");
            if (ompThreads != NULL)
            {
                mls.setNumberOfThreads(atoi(ompThreads));
            }
            else {
                std::cout << QString("WARNING: OMP selected, but OMP_NUM_THREADS not set - using automatic detection") + "\n";
            }
            //For time being, we will make sure we only calc normals, maybe not later.  
            mls.setComputeNormals(true);        
            //Set input parameters
            mls.setInputCloud(cloud);
            if (*polyFit_ == TRUE)
            {
                mls.setPolynomialFit(true);
                mls.setPolynomialOrder(*polyOrder_);
            }
            mls.setSearchMethod(tree);
            mls.setSearchRadius(*searchRadius_);
            //Reconstruct
            mls.process(mls_cloud); 
        }
        else {
            // Init object (second point type is for the normals, even if unused)
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
            //For time being, we will make sure we only calc normals, maybe not later.  
            mls.setComputeNormals(true);        
            //Set input parameters
            mls.setInputCloud(cloud);
            if (*polyFit_ == TRUE)
            {
                mls.setPolynomialFit(true);
                mls.setPolynomialOrder(*polyOrder_);
            }
            mls.setSearchMethod(tree);
            mls.setSearchRadius(*searchRadius_);
            //Reconstruct
            mls.process(mls_cloud); 
        }
        //Now add the filtered nodes to the output mesh model

        int a = 0;
        NodeStateHandle rgbahandle = outputNodes.addState("RGBA", a);

        for (size_t i = 0; i < mls_cloud.points.size (); ++i)
        {
            Vector3d vo,no;
            vo.x = mls_cloud.points[i].x;
            vo.y = mls_cloud.points[i].y;
            vo.z = mls_cloud.points[i].z;
            no.x = mls_cloud.points[i].normal_x;
            no.y = mls_cloud.points[i].normal_y;
            no.z = mls_cloud.points[i].normal_z;
            NodeHandle node = outputNodes.add(vo);
            outputNodes.setState(node,normalStateOut,no);
            outputNodes.setState(node,curvatureStateOut,mls_cloud.points[i].curvature);
        }

        return true;
    }


    /**
     *
     */
    MovingLeastSquares::MovingLeastSquares() :
        DataExecution::Operation(
            DataExecution::OperationFactoryTraits< MovingLeastSquares >::getInstance(),
            tr("Moving least squares smoothing"))
    {
        pImpl_ = new MovingLeastSquaresImpl(*this);
    }


    /**
     *
     */
    MovingLeastSquares::~MovingLeastSquares()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  MovingLeastSquares::execute()
    {
        return pImpl_->execute();
    }
}
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(MovingLeastSquares, 
                                   PointCloud::PointCloudPlugin::getInstance(),
                                   DataExecution::Operation::tr("PointCloud"))
