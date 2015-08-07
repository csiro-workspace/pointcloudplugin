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
#include "removestatisticaloutliers.h"

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{

    /**
     * \internal
     */
    class RemoveStatisticalOutliersImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RemoveStatisticalOutliersImpl)

    public:
        RemoveStatisticalOutliers&  op_;

        // Data objects
        DataExecution::TypedObject< MeshModelInterface >  dataMesh_;
        DataExecution::TypedObject< double >              numberofNeighbours_;
        DataExecution::TypedObject< double >              sdevThreshold_;


        // Inputs and outputs
        DataExecution::InputScalar inputMesh_;
        DataExecution::InputScalar inputnumberofNeighbours_;
        DataExecution::InputScalar inputsdevThreshold_;
        DataExecution::Output      outputMesh_;


        RemoveStatisticalOutliersImpl(RemoveStatisticalOutliers& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    RemoveStatisticalOutliersImpl::RemoveStatisticalOutliersImpl(RemoveStatisticalOutliers& op) :
        op_(op),
        dataMesh_(),
        numberofNeighbours_(50),
        sdevThreshold_(1.0),
        inputMesh_("Points", dataMesh_, op_, true),
        inputnumberofNeighbours_("Number of Neighbours to search", numberofNeighbours_, op_),
        inputsdevThreshold_("Standard Deviation Threshold", sdevThreshold_, op_),
        outputMesh_("Points", dataMesh_, op_)
    {
        inputnumberofNeighbours_.setDescription("Sets the number of points to use for mean distance estimation");
        inputsdevThreshold_.setDescription("Sets the standard deviation threshold. All points outside the mean+-STD*multiplier threshold will be considered outliers.");
    }


    /**
     *
     */
    bool RemoveStatisticalOutliersImpl::execute()
    {
        MeshModelInterface& mesh = *dataMesh_;
        MeshNodesInterface& nodes = mesh.getNodes();
        
        const NodeStateHandle& keepNodeState = nodes.addState<MeshModelInterface::int_type>("tmp_keep", 0);

        // For each node add point to point cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        MeshNodesInterface::const_iterator nIter = nodes.begin();
        MeshNodesInterface::const_iterator end = nodes.end();
        for(; nIter != end; ++nIter)
        {
           Vector3d v = nodes.getPosition(*nIter);
           pcl::PointXYZRGBA p;
           p.x = v.x;
           p.y = v.y;
           p.z = v.z;
           p.rgba = nIter->getIndex();  // store index in rgba
           cloud->push_back(pcl::PointXYZRGBA(p));
        }

        // Create the filtered point cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (*numberofNeighbours_);
        sor.setStddevMulThresh (*sdevThreshold_);
        sor.filter (*cloud_filtered);
    
        // Mark all the nodes that still exist in the filtered cloud
        for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
        {
            NodeHandle n(cloud_filtered->points[i].rgba);
            nodes.setState(n, keepNodeState, 1);
        }

        // Delete any points not marked to keep
        MeshModelInterface::int_type keep;
        nIter = nodes.begin();
        for(; nIter != end; ++nIter)
        {
            nodes.getState(*nIter, keepNodeState, keep);
            if (!keep)
            {
                nodes.remove(*nIter);
            }
        }

        nodes.removeState(keepNodeState);
        mesh.emptyTrash();
        
        return true;
    }


    /**
     *
     */
    RemoveStatisticalOutliers::RemoveStatisticalOutliers() :
        DataExecution::Operation(
            DataExecution::OperationFactoryTraits< RemoveStatisticalOutliers >::getInstance(),
            tr("Remove Statistical Outliers"))
    {
        pImpl_ = new RemoveStatisticalOutliersImpl(*this);
    }


    /**
     *
     */
    RemoveStatisticalOutliers::~RemoveStatisticalOutliers()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  RemoveStatisticalOutliers::execute()
    {
        return pImpl_->execute();
    }
}
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(RemoveStatisticalOutliers, 
                                   PointCloud::PointCloudPlugin::getInstance(),
                                   DataExecution::Operation::tr("PointCloud"))
