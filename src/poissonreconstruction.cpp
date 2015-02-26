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
#include <vector>

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
#include "poissonreconstruction.h"

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

    namespace PointCloud
    {

        /**
        * \internal
        */
        class PoissonReconstructionImpl
        {
            // Allow string translation to work properly
            Q_DECLARE_TR_FUNCTIONS(PoissonReconstructionImpl)

        public:
            PoissonReconstruction&  op_;

            // Data objects
            DataExecution::TypedObject< MeshModelInterface >  dataMesh_;
            DataExecution::TypedObject< MeshModelInterface >  dataOutputMesh_;
            DataExecution::TypedObject< int >                 treeDepth_;
            DataExecution::TypedObject< int >                 minTreeDepth_;
            DataExecution::TypedObject< double >              pointWeight_;
            DataExecution::TypedObject< double >              setScale_;
            DataExecution::TypedObject< int >                 solverDivide_;
            DataExecution::TypedObject< int >                 isoDivide_;
            DataExecution::TypedObject< double >              samplesPerNode_;
            DataExecution::TypedObject< bool >                setConfidence_;
            DataExecution::TypedObject< bool >                setPolygons_;
            DataExecution::TypedObject< bool >                setManifold_;
            DataExecution::TypedObject< int >                setDegree_;
            DataExecution::TypedObject< QString >             normalName_;
            DataExecution::TypedObject< QString >             curvName_;

            // Inputs and outputs
            DataExecution::InputScalar inputMesh_;
            DataExecution::InputScalar inputtreeDepth_;
            DataExecution::InputScalar inputminTreeDepth_;
            DataExecution::InputScalar inputpointWeight_;
            DataExecution::InputScalar inputsetScale_;
            DataExecution::InputScalar inputsolverDivide_;
            DataExecution::InputScalar inputisoDivide_;
            DataExecution::InputScalar inputsamplesPerNode_;
            DataExecution::InputScalar inputsetConfidence_;
            DataExecution::InputScalar inputsetPolygons_;
            DataExecution::InputScalar inputsetManifold_;
            DataExecution::InputScalar inputsetDegree_;
            DataExecution::InputScalar inputnormalName_;
            DataExecution::InputScalar inputcurvName_; 
            DataExecution::Output      outputMesh_;


            PoissonReconstructionImpl(PoissonReconstruction& op);

            bool  execute();
            void  logText(const QString& msg)   { op_.logText(msg); }
        };


        /**
        *
        */
        PoissonReconstructionImpl::PoissonReconstructionImpl(PoissonReconstruction& op) :
        op_(op),
            dataMesh_(),
            dataOutputMesh_(),
            treeDepth_(9),
            minTreeDepth_(5),
            pointWeight_(4.0),
            setScale_(1.1),
            solverDivide_(8),
            isoDivide_(8),
            samplesPerNode_(5.0),
            setConfidence_(false),
            setPolygons_(true),
            setManifold_(false),
            setDegree_(2),
            normalName_("normal"),
            curvName_("curvature"),
            inputMesh_("Points", dataMesh_, op_),
            inputtreeDepth_("Reconstruction octree depth", treeDepth_, op_),
            inputminTreeDepth_("Minimum octree depth", minTreeDepth_, op_),
            inputpointWeight_("Interpolation weight", pointWeight_, op_),
            inputsetScale_("Scale factor", setScale_, op_),
            inputsolverDivide_("Solver subdivision depth", solverDivide_, op_),
            inputisoDivide_("Iso-surface extraction subdivision depth", isoDivide_, op_),
            inputsamplesPerNode_("Minimum number of samples", samplesPerNode_, op_),
            inputsetConfidence_("Confident in normals", setConfidence_, op_),
            inputsetPolygons_("Polygon mesh output", setPolygons_, op_),
            inputsetManifold_("Manifold mesh", setManifold_, op_),
            inputsetDegree_("Degree paramater", setDegree_, op_),
            inputnormalName_("Normal state name", normalName_, op_),
            inputcurvName_("Curvature state name", curvName_, op_),
            outputMesh_("Mesh Model", dataOutputMesh_, op_)
        {
            inputtreeDepth_.setDescription("This is the maximum depth of the tree that will be used for surface reconstruction.\
                                           Running at depth d corresponds to solving on a voxel grid whose resolution is no larger than 2^d\
                                           x 2^d x 2^d. Note that since the reconstructor adapts the octree to the sampling density, the specified\
                                           reconstruction depth is only an upper bound.");
            inputminTreeDepth_.setDescription("This specifies the depth beyond depth the octree will be adapted. At coarser depths, the octree\
                                              will be complete, containing all 2^d x 2^d x 2^d nodes");
            inputpointWeight_.setDescription("This value specifies the importants that interpolation of the point samples is given in the\
                                             formulation of the screened Poisson equation. The results of the original (unscreened) Poisson Reconstruction\
                                             can be obtained by setting this value to 0.");
            inputsetScale_.setDescription("This floating point value specifies the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube");
            inputsolverDivide_.setDescription("This integer argument specifies the depth at which a block Gauss-Seidel solver is used to solve the Laplacian\
                                              equation. Using this parameter helps reduce the memory overhead at the cost of a small increase in reconstruction\
                                              time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly\
                                              reduce the memory usage.)");
            inputisoDivide_.setDescription("This integer argument specifies the depth at which a block iso-surface extractor should be used to extract the iso-surface.\
                                           Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. (In practice, we have\
                                           found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)");
            inputsamplesPerNode_.setDescription("This floating point value specifies the minimum number of sample points that should fall within an octree node\
                                                as the octree construction is adapted to sampling density. For noise-free samples, small values in the range [1.0 - 5.0]\
                                                can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.");
            inputsetConfidence_.setDescription("Enabling this flag tells the reconstructor to use the size of the normals as confidence information. When the flag is not enabled, all normals are\
                                               normalized to have unit-length prior to reconstruction.");
            inputsetPolygons_.setDescription("Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating the results of Marching Cubes).");
            inputsetManifold_.setDescription("Enabling this flag tells the reconstructor to add the polygon barycenter when triangulating polygons with more than three vertices.");

        }


        /**
        *
        */
        bool PoissonReconstructionImpl::execute()
        {
            MeshModelInterface& mesh = *dataMesh_;
            MeshModelInterface& outputMesh = *dataOutputMesh_;

            MeshNodesInterface& nodes = mesh.getNodes();
            outputMesh.clear();
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
            //Warn users about how terrible this is
            std::cout << QString("WARNING: This is quite possibly the worst implementation of poisson surface reconstruction \
                                 PCL 1.7.1 still does not fully support all the options required. Use at your own risk") + "\n";

            const NodeStateHandle& normalStateOut = outputNodes.addState("normal", Vector3d(0,0,-1));

            if(nodes.size() == 0)
            {
                return true;
            }

            pcl::Poisson<pcl::PointNormal> poisson;
            //Set all Poisson values 
            poisson.setConfidence(*setConfidence_);
            poisson.setDegree(*setDegree_);
            poisson.setDepth(*treeDepth_);
            poisson.setIsoDivide(*isoDivide_);
            poisson.setManifold(*setManifold_);
            poisson.setOutputPolygons(*setPolygons_);
            poisson.setSamplesPerNode(*samplesPerNode_);
            poisson.setScale(*setScale_);
            poisson.setSolverDivide(*solverDivide_);

            if (!nodes.hasState(*normalName_) && !nodes.hasState(*curvName_))
            {
                std::cout << QString("ERROR: Model does not have states named %1 and %2. Ensure normals and curvatures are pre-calculated").arg(*normalName_).arg(*curvName_) + "\n";
                return false;
            }	

            // Input has the PointNormal type in order to store the normals
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

            //Add nodes to the pointcloud
            MeshNodesInterface::const_iterator nIter = nodes.begin();
            MeshNodesInterface::const_iterator end = nodes.end();
            NodeStateHandle innodeNormals = nodes.getStateHandle(*normalName_);
            NodeStateHandle innodeCurvature = nodes.getStateHandle(*curvName_);
            for(; nIter != end; ++nIter)
            {
                Vector3d v = nodes.getPosition(*nIter);
                Vector3d n;
                nodes.getState(*nIter,innodeNormals,n);
                double c;
                nodes.getState(*nIter,innodeCurvature,c);	
                //Add point,normals and curvature 
                pcl::PointNormal p;
                p.x = v.x;
                p.y = v.y;
                p.z = v.z;
                p.normal_x = n.x;
                p.normal_y = n.y;
                p.normal_z = n.z;
                p.curvature = c;
                cloud_with_normals->push_back(pcl::PointNormal(p));
            }

            //Initialise polymesh
            poisson.setInputCloud(cloud_with_normals);
            pcl::PointCloud<pcl::PointNormal> poisspoints;
            std::vector<pcl::Vertices> verts;
            //poisson.reconstruct(triangles);
            poisson.reconstruct(poisspoints,verts);
            std::cout << QString("Poisson construction completed, writing to datastructure.") + "\n";

            // Allocate DataObjects of the correct types to copy the state data with.
            // We also re-populate srcNodeStates to match dstNodeStates since this may
            // be less than than all the src states if allowPartialAssign is true. It's 
            // also possible the dst uses a different order.
            NodeStateHandleList srcNodeStates = nodes.getAllStateHandles();
            NodeStateHandleList dstNodeStates;
            QList<DataObject*> nodeStateData;
            foreach(const NodeStateHandle* state, srcNodeStates)
            {
                nodeStateData.push_back(state->getDataFactory().createDataObject());
                if (!outputNodes.hasState(state->getName()))
                    outputNodes.addState(state->getName(), state->getDefaultValue());

                dstNodeStates.push_back(&outputNodes.getStateHandle(state->getName()));
                assert(dstNodeStates.back()->isValid());
            }

            int numNodeStates = srcNodeStates.size();
            size_t numPointsInCloud = poisspoints.size();
            size_t numTris = verts.size();
            
            QVector<bool> nodeHasTri(static_cast<int>(numPointsInCloud), false);
            for(int i = 0; i < numTris; ++i)
            {
                std::vector<uint32_t>& vertices = verts[i].vertices;
                nodeHasTri[vertices[0]] = true;
                nodeHasTri[vertices[1]] = true;
                nodeHasTri[vertices[2]] = true;
            }

            //Adds vertices to output mesh model
            //Required as MLS method of reconstruction may change vertex position
            QVector<NodeHandle> nodeLookup(numPointsInCloud);
            MeshNodesInterface::const_iterator nnIter = nodes.begin();
            for(int i = 0; i < numPointsInCloud; ++i)
            {
                if (nodeHasTri[i])
                {
                    //pcl::PointNormal& p =cloud_with_normals->at(i);
                    pcl::PointNormal& p = poisspoints.at(i);
                    NodeHandle nhnd = outputNodes.add(Vector3d(p.x,p.y,p.z));
                    nodeLookup[i] = nhnd;

                    outputNodes.setState(nhnd, normalStateOut, Vector3d(p.normal_x, p.normal_y, p.normal_z).unitVector());

                    // copy all the states for this node
                    for (int iState=0; iState<numNodeStates; ++iState)
                    {
                        bool ok = nodes.getState(*nnIter, *srcNodeStates[iState], *nodeStateData[iState]);
                        assert(ok);
                        ok = outputNodes.setState(nhnd, *dstNodeStates[iState], *nodeStateData[iState]);
                        assert(ok);
                    }

#if 0 // old kinect stuff

                    // Setup texture coordinates based on the pixel location
                    if (pixIdStateIn)
                    {
                        MeshModelInterface::int_type pixId;
                        nodes.getState(*nIter, *pixIdStateIn, pixId);
                        outputNodes.setState(nhnd, *textureSStateOut, -double(pixId%HardwareManager::SensorResolutionX) / HardwareManager::SensorResolutionX);
                        outputNodes.setState(nhnd, *textureTStateOut, -double(pixId/HardwareManager::SensorResolutionX) / HardwareManager::SensorResolutionY);
                    }
#endif
                }
                ++nnIter;
            }

            // Delete the node state data objects used for copying states
            foreach(DataObject* dataObject, nodeStateData)
            {
                dataObject->getFactory().destroyDataObject(dataObject);
            }
            nodeStateData.clear();

            MeshElementsInterface& tris = outputMesh.getElements(ElementType::Tri::getInstance());

            //For each triangle in the PolygonMesh add tri to mesh model
            for(int i = 0; i < numTris; ++i)
            {
                //Accesses vertices array of each triangle
                std::vector<uint32_t>& vertices = verts[i].vertices;
                tris.add(nodeLookup[vertices[0]],nodeLookup[vertices[1]],nodeLookup[vertices[2]]);
            }

            return true;
        }


        /**
        *
        */
        PoissonReconstruction::PoissonReconstruction() :
        DataExecution::Operation(
            DataExecution::OperationFactoryTraits< PoissonReconstruction >::getInstance(),
            tr("Reconstruct Surface (Poisson)"))
        {
            pImpl_ = new PoissonReconstructionImpl(*this);
        }


        /**
        *
        */
        PoissonReconstruction::~PoissonReconstruction()
        {
            delete pImpl_;
        }


        /**
        *
        */
        bool  PoissonReconstruction::execute()
        {
            return pImpl_->execute();
        }
    }
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(PoissonReconstruction, 
    PointCloud::PointCloudPlugin::getInstance(),
    DataExecution::Operation::tr("PointCloud"))
