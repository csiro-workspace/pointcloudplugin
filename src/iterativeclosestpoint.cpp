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
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"
#include "Mesh/Geometry/transformmatrix.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "pointcloudplugin.h"
#include "iterativeclosestpoint.h"


namespace CSIRO
{

namespace PointCloud
{
    /**
     * \internal
     */
    using namespace Mesh;

    class IterativeClosestPointImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::IterativeClosestPointImpl)

    public:
        IterativeClosestPoint&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  dataModelTarget_;
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  dataModelSource_;
        CSIRO::DataExecution::TypedObject< int >                              dataMaximumIterations_;
        CSIRO::DataExecution::TypedObject< double >                           dataConvergenceLimit_;
        CSIRO::DataExecution::TypedObject< double >                           dataEuclideanDistanceConvergenceLimit_;
        CSIRO::DataExecution::TypedObject< double >                           dataMaximumCorrespondenceDistance_;
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::TransformMatrix >      dataTransform_;


        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputModelTarget_;
        CSIRO::DataExecution::InputScalar inputModelSource_;
        CSIRO::DataExecution::InputScalar inputMaximumIterations_;
        CSIRO::DataExecution::InputScalar inputConvergenceLimit_;
        CSIRO::DataExecution::InputScalar inputEuclideanDistanceConvergenceLimit_;
        CSIRO::DataExecution::InputScalar inputMaximumCorrespondenceDistance_;
        CSIRO::DataExecution::Output      outputTransform_;


        IterativeClosestPointImpl(IterativeClosestPoint& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    IterativeClosestPointImpl::IterativeClosestPointImpl(IterativeClosestPoint& op) :
        op_(op),
        dataModelTarget_(),
        dataModelSource_(),
        dataMaximumIterations_(500),
        dataConvergenceLimit_(1e-8),
        dataEuclideanDistanceConvergenceLimit_(1e-8),
        dataMaximumCorrespondenceDistance_(5),
        dataTransform_(),
        inputModelTarget_("Model target", dataModelTarget_, op_),
        inputModelSource_("Model source", dataModelSource_, op_),
        inputMaximumIterations_("Maximum iterations", dataMaximumIterations_, op_),
        inputConvergenceLimit_("Epsilon convergence limit", dataConvergenceLimit_, op_),
        inputEuclideanDistanceConvergenceLimit_("Euclidean distance convergence limit", dataEuclideanDistanceConvergenceLimit_, op_),
        inputMaximumCorrespondenceDistance_("Maximum correspondence distance", dataMaximumCorrespondenceDistance_, op_),
        outputTransform_("Output Transform", dataTransform_, op_)
    {
        inputMaximumIterations_.setDescription("Set the maximum number of iterations the internal optimization should run for");
        inputConvergenceLimit_.setDescription("Set the transformation epsilon (maximum allowable difference between two \
                                              consecutive transformations) in order for an optimization to be considered as having converged to the final solution.");
        inputEuclideanDistanceConvergenceLimit_.setDescription("Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged.");
        inputMaximumCorrespondenceDistance_.setDescription("Set the maximum distance threshold between two correspondent points in source <-> target");
    }


    /**
     *
     */
   void printMatix4f(const Eigen::Matrix4f & matrix) {

	std::cout << QString("Rotation matrix :") + "\n";
	std::cout << QString("    | %1 %2 %3 |      ").arg(matrix (0,0)).arg(matrix (0,1)).arg(matrix (0,2)) + "\n";
	std::cout << QString("R = | %1 %2 %3 | ").arg(matrix (1,0)).arg(matrix (1,1)).arg(matrix (1,2)) + "\n";
	std::cout << QString("    | %6.3f %6.3f %6.3f | ").arg(matrix (2,0)).arg(matrix (2,1)).arg(matrix (2,2)) + "\n";
	std::cout << QString("Translation vector :")+"\n";
	std::cout << QString("t = < %1, %2, %2 >").arg(matrix (0,3)).arg(matrix (1,3)).arg(matrix (2,3)) + "\n";
}
    bool IterativeClosestPointImpl::execute()
    {
        CSIRO::Mesh::MeshModelInterface& modelTarget                       = *dataModelTarget_;
        CSIRO::Mesh::MeshModelInterface& modelSource                       = *dataModelSource_;
        int&                             maximumIterations                 = *dataMaximumIterations_;
        double&                          convergenceLimit                  = *dataConvergenceLimit_;
        double&                          euclideanDistanceConvergenceLimit = *dataEuclideanDistanceConvergenceLimit_;
        double&                          maximumCorrespondenceDistance     = *dataMaximumCorrespondenceDistance_;
        CSIRO::Mesh::TransformMatrix&     transform                         = *dataTransform_;
        
        //Get model and nodes for source and target
        MeshNodesInterface& targetNodes = modelTarget.getNodes();
        MeshNodesInterface& sourceNodes = modelSource.getNodes();

        //Create the pcl point clounds (as we only get the transfrom, can ignore everything but XYZ)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget (new pcl::PointCloud<pcl::PointXYZ>);
        for (MeshNodesInterface::const_iterator tIter = targetNodes.begin(); tIter != targetNodes.end(); ++tIter)
        {
            Vector3d v = targetNodes.getPosition(*tIter);
            pcl::PointXYZ p;
            p.x = v.x;
            p.y = v.y;
            p.z = v.z;
            cloudTarget->push_back(pcl::PointXYZ(p));
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZ>);
        for (MeshNodesInterface::const_iterator sIter = sourceNodes.begin(); sIter != sourceNodes.end(); ++sIter)
        {
            Vector3d v = sourceNodes.getPosition(*sIter);
            pcl::PointXYZ p;
            p.x = v.x;
            p.y = v.y;
            p.z = v.z;
            cloudSource->push_back(pcl::PointXYZ(p));
        }

        //Create ICP instance
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        //Input sources and targets
        icp.setInputCloud(cloudSource);
        icp.setInputTarget(cloudTarget);
        //Settings
        icp.setMaxCorrespondenceDistance(maximumCorrespondenceDistance);
        icp.setMaximumIterations(maximumIterations);
        icp.setTransformationEpsilon(convergenceLimit);
        icp.setEuclideanFitnessEpsilon(euclideanDistanceConvergenceLimit);
        //Run ICP
        std::cout << QString("Point clouds allocated, beginning ICP...") + "\n";
        //pcl::PointCloud<pcl::PointXYZ> aligncloud;
        icp.align(*cloudSource);
        std::cout << QString("ICP alignment completed, fitness score is %1").arg(icp.getFitnessScore()) + "\n";
        //Set transform
        Eigen::Matrix4f transformationmatrix = icp.getFinalTransformation();
        printMatix4f(transformationmatrix);
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                transform(r,c) = transformationmatrix(r,c);
            }
        }
       

        return true;
    }


    /**
     *
     */
    IterativeClosestPoint::IterativeClosestPoint() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< IterativeClosestPoint >::getInstance(),
            tr("Iterative closest point model transform"))
    {
        pImpl_ = new IterativeClosestPointImpl(*this);
    }


    /**
     *
     */
    IterativeClosestPoint::~IterativeClosestPoint()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  IterativeClosestPoint::execute()
    {
        return pImpl_->execute();
    }
}}


using namespace CSIRO::PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(IterativeClosestPoint, 
                                   CSIRO::PointCloud::PointCloudPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PointCloud"))

