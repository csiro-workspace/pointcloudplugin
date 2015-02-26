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
#include <vector>
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

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include "pointcloudplugin.h"
#include "normalestimation.h"

namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{

    /**
     * \internal
     */
    class NormalEstimationImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(NormalEstimationImpl)

    public:
        NormalEstimation&  op_;

        // Data objects
        DataExecution::TypedObject< MeshModelInterface >    dataMesh_;
        DataExecution::TypedObject< bool >                  dataOMP_;
        DataExecution::TypedObject< Vector3dGroup >         dataViewpoint_;
        DataExecution::TypedObject< double >                dataSearchRadius_;
        DataExecution::TypedObject< bool >                  dataReverse_;
        DataExecution::TypedObject< MeshModelInterface >    dataOutputMesh_; //Should be able to do it in place, but not at this stage


        // Inputs and outputs
        DataExecution::InputScalar inputMesh_;
        DataExecution::InputScalar inputOMP_;
        DataExecution::InputScalar inputViewpoint_;
        DataExecution::InputScalar inputSearchRadius_;
        DataExecution::InputScalar inputReverse_;
        DataExecution::Output      outputMesh_;

        NormalEstimationImpl(NormalEstimation& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    NormalEstimationImpl::NormalEstimationImpl(NormalEstimation& op) :
        op_(op),
        dataMesh_(),
        dataOMP_(false),
        dataOutputMesh_(),
        dataViewpoint_(Vector3d(0,0,0)),
        dataSearchRadius_(0.1),
        dataReverse_(false),
        inputMesh_("Input Mesh", dataMesh_, op_),
        inputOMP_("Use OpenMP", dataOMP_, op_),
        inputViewpoint_("Viewpoint Location", dataViewpoint_, op_),
        inputSearchRadius_("Node search radius", dataSearchRadius_, op_),
        inputReverse_("Reverse normals", dataReverse_, op_),
        outputMesh_("Mesh Model", dataOutputMesh_, op_)
    {
        inputOMP_.setDescription("Enables OpenMP based computation, threads will be determined on the OMP_NUM_THREADS environment variable");
    }
        

    /**
     *
     */
    bool NormalEstimationImpl::execute()
    {
        //Get model and nodes
        MeshModelInterface& mesh = *dataMesh_;
        MeshNodesInterface& nodes = mesh.getNodes();
        MeshModelInterface& outputMesh = *dataOutputMesh_;
        outputMesh.clear();//Always clear output mesh
        MeshNodesInterface& outputNodes = outputMesh.getNodes();
        //Get node states
        if (!nodes.hasState("RGBA"))
        {
            std::cout << QString("WARNING: Nodes do not have RGBA state") + "\n";
            nodes.addState<int>("RGBA",0);
        }
        NodeStateHandle inputRgba = nodes.getStateHandle("RGBA");
      
                
        //For each node add point to point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//There is no xyzrgbaNormal, so have to use rgb
        MeshNodesInterface::const_iterator nIter = nodes.begin();
        MeshNodesInterface::const_iterator end = nodes.end();
        for(; nIter != end; ++nIter)
        {
           int rgba = 0; 
           Vector3d v = nodes.getPosition(*nIter);
           nodes.getState(*nIter,inputRgba,rgba);//Will return false if no rgba state - assert?
           pcl::PointXYZRGB p;
           p.x = v.x;
           p.y = v.y;
           p.z = v.z;
           p.rgba = rgba;   
           cloud->push_back(pcl::PointXYZRGB(p));
        }
        
        //Create search tree
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        //Setup output dataset
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        
        //Create PCL normal estimator
        if (*dataOMP_)
        {
            pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
            char * ompThreads;
            ompThreads = getenv("OMP_NUM_THREADS");
            if (ompThreads != NULL)
            {
                ne.setNumberOfThreads(atoi(ompThreads));
            }
            else {
                std::cout << QString("WARNING: OMP selected, but OMP_NUM_THREADS not set - using automatic detection") + "\n";
            }
            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            //Set search radius
            ne.setRadiusSearch(*dataSearchRadius_);
            ne.setViewPoint(*dataViewpoint_->x,*dataViewpoint_->y,*dataViewpoint_->z);
            //Compute normals
            ne.compute(*cloud_normals);
        }
        else
        {
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            //Set search radius
            ne.setRadiusSearch(*dataSearchRadius_);
            ne.setViewPoint(*dataViewpoint_->x,*dataViewpoint_->y,*dataViewpoint_->z);
            //Compute normals
            ne.compute(*cloud_normals);
        }

        //Reverse if asked
        if (*dataReverse_)
        {
            for (size_t i = 0; i < cloud_normals->size (); ++i)
             {
                 cloud_normals->points[i].normal_x *= -1;
                 cloud_normals->points[i].normal_y *= -1;
                 cloud_normals->points[i].normal_z *= -1;
             }
        }

        //Now combine clouds
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals_comb (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
        pcl::concatenateFields(*cloud, *cloud_normals, *cloud_normals_comb);
        
        //Add states for normal, curvature and rgba
        const NodeStateHandle& normalStateOut = outputNodes.addState("normal", Vector3d(0,0,-1));
        const NodeStateHandle& curvatureStateOut = outputNodes.addState<double>("curvature",0.0);
        int a = 0;
        NodeStateHandle rgbahandle = outputNodes.addState("RGBA", a);
        //Now add the filtered nodes to the output mesh model
        for (size_t i = 0; i < cloud_normals_comb->points.size (); ++i)
        {
            Vector3d vo,no;
            vo.x = cloud_normals_comb->points[i].x;
            vo.y = cloud_normals_comb->points[i].y;
            vo.z = cloud_normals_comb->points[i].z;
            no.x = cloud_normals_comb->points[i].normal_x;
            no.y = cloud_normals_comb->points[i].normal_y;
            no.z = cloud_normals_comb->points[i].normal_z;
            NodeHandle node = outputNodes.add(vo);
            int alph = static_cast<int> (cloud_normals_comb->points[i].rgba);
            outputNodes.setState(node,normalStateOut,no);
            outputNodes.setState(node,curvatureStateOut,cloud_normals_comb->points[i].curvature);
            outputNodes.setState(node,rgbahandle,alph);
        }

        return true;
    }


    /**
     *
     */
    NormalEstimation::NormalEstimation() :
        DataExecution::Operation(
            DataExecution::OperationFactoryTraits< NormalEstimation >::getInstance(),
            tr("Normal estimation and adjustment"))
    {
        pImpl_ = new NormalEstimationImpl(*this);
    }


    /**
     *
     */
    NormalEstimation::~NormalEstimation()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  NormalEstimation::execute()
    {
        return pImpl_->execute();
    }
}
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(NormalEstimation, 
                                   PointCloud::PointCloudPlugin::getInstance(),
                                   DataExecution::Operation::tr("PointCloud"))
