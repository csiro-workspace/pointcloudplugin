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

#include "recon-ssd/Settings.hpp"
#include "octree/OctreeBundle.hpp"
#include "isosurface/DualMarchingCubes.hpp"
#include "dgp/PointReader.hpp"
#include "dgp/PlyWriter.hpp"
#include "recon-ssd/run_ssd.hpp"

#include "pointcloudplugin.h"
#include "smoothesigneddistance.h"

namespace CSIRO
{

namespace PointCloud
{
    /**
     * \internal
     */
    class SmootheSignedDistanceImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::SmootheSignedDistanceImpl)

    public:
        SmootheSignedDistance&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  dataPointCloud_;
        CSIRO::DataExecution::TypedObject< bool >                             dataVerbose_;
        CSIRO::DataExecution::TypedObject< bool >                             dataManifold_;
        CSIRO::DataExecution::TypedObject< bool >                             dataComputeColor_;
        CSIRO::DataExecution::TypedObject< double >                           dataBboxExpFactor_;
        CSIRO::DataExecution::TypedObject< int >                              dataOctreeLevels_;
        CSIRO::DataExecution::TypedObject< int >                              dataOctreeStartLevel_;
        CSIRO::DataExecution::TypedObject< int >                              dataSamplesPerNode_;
        CSIRO::DataExecution::TypedObject< double >                           dataIsolevel_;
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::Vector3dGroup >       dataWeight_;
        CSIRO::DataExecution::TypedObject< double >                           dataColorWeight0_;
        CSIRO::DataExecution::TypedObject< double >                           dataColorWeight1_;
        CSIRO::DataExecution::TypedObject< double >                           dataTolerance_;
        CSIRO::DataExecution::TypedObject< double >                           dataColorTolerance_;
        CSIRO::DataExecution::TypedObject< int >                              dataMinimumIterations_;
        CSIRO::DataExecution::TypedObject< int >                              dataMaximumIterations_;
        CSIRO::DataExecution::TypedObject< bool >                             dataFastConstruction_;
        CSIRO::DataExecution::TypedObject< bool >                             dataMakePolygons_;
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  dataOutputMesh_;


        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputPointCloud_;
        CSIRO::DataExecution::InputScalar inputVerbose_;
        CSIRO::DataExecution::InputScalar inputManifold_;
        CSIRO::DataExecution::InputScalar inputComputeColor_;
        CSIRO::DataExecution::InputScalar inputBboxExpFactor_;
        CSIRO::DataExecution::InputScalar inputOctreeLevels_;
        CSIRO::DataExecution::InputScalar inputOctreeStartLevel_;
        CSIRO::DataExecution::InputScalar inputSamplesPerNode_;
        CSIRO::DataExecution::InputScalar inputIsolevel_;
        CSIRO::DataExecution::InputScalar inputWeight_;
        CSIRO::DataExecution::InputScalar inputColorWeight0_;
        CSIRO::DataExecution::InputScalar inputColorWeight1_;
        CSIRO::DataExecution::InputScalar inputTolerance_;
        CSIRO::DataExecution::InputScalar inputColorTolerance_;
        CSIRO::DataExecution::InputScalar inputMinimumIterations_;
        CSIRO::DataExecution::InputScalar inputMaximumIterations_;
        CSIRO::DataExecution::InputScalar inputFastConstruction_;
        CSIRO::DataExecution::InputScalar inputMakePolygons_;
        CSIRO::DataExecution::Output      OdataOutputMesh_;


        SmootheSignedDistanceImpl(SmootheSignedDistance& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    SmootheSignedDistanceImpl::SmootheSignedDistanceImpl(SmootheSignedDistance& op) :
        op_(op),
        dataPointCloud_(),
        dataVerbose_(false),
        dataManifold_(false),
        dataComputeColor_(false),
        dataBboxExpFactor_(1.1),
        dataOctreeLevels_(8),
        dataOctreeStartLevel_(1),
        dataSamplesPerNode_(1),
        dataIsolevel_(0.0),
        dataWeight_(Mesh::Vector3d(1,1,1)),
        dataColorWeight0_(1),
        dataColorWeight1_(0.1),
        dataTolerance_(1.0e-8),
        dataColorTolerance_(1.0e-4),
        dataMinimumIterations_(1),
        dataMaximumIterations_(300),
        dataFastConstruction_(false),
        dataMakePolygons_(false),
        dataOutputMesh_(),
        inputPointCloud_("Point Cloud", dataPointCloud_, op_),
        inputVerbose_("Verbose logging", dataVerbose_,op_),
        inputManifold_("Manifold mesh (experimental)", dataManifold_, op_),
        inputComputeColor_("Compute Color", dataComputeColor_, op_),
        inputBboxExpFactor_("Bounding Box Expansion Factor", dataBboxExpFactor_, op_),
        inputOctreeLevels_("Octree Levels", dataOctreeLevels_, op_),
        inputOctreeStartLevel_("Octree Start Level", dataOctreeStartLevel_, op_),
        inputSamplesPerNode_("Samples per node", dataSamplesPerNode_, op_),
        inputIsolevel_("Isolevel", dataIsolevel_, op_),
        inputWeight_("Weight", dataWeight_, op_),
        inputColorWeight0_("Color weight 1", dataColorWeight0_, op_),
        inputColorWeight1_("Color weight 2", dataColorWeight1_, op_),
        inputTolerance_("Tolerance", dataTolerance_, op_),
        inputColorTolerance_("Color Tolerance", dataColorTolerance_, op_),
        inputMinimumIterations_("Minimum Iterations", dataMinimumIterations_, op_),
        inputMaximumIterations_("Maximum Iterations", dataMaximumIterations_, op_),
        inputFastConstruction_("Fast Construction", dataFastConstruction_, op_),
        inputMakePolygons_("Make Polygons", dataMakePolygons_, op_),
        OdataOutputMesh_("Output Mesh", dataOutputMesh_, op_)
    {
        inputManifold_.setDescription("An experimental setting in SSD reconstruction, which dictates whether the mesh should \
                                        be manifold. Calls DualMarchingCubes::PolygonToManifoldTriangleMesh, but may or may not work.");
        inputOctreeLevels_.setDescription("This positive integer is the maximum level of the octree that will be \
	                                        used for surface reconstruction. Running at level d corresponds to solving \
	                                        on a voxel grid whose resolution is no larger than 2^d x 2^d x 2^d.");
        inputWeight_.setDescription("These positive floating point values specifies regularization \
	                                    parameters presented in the paper. We suggest using smaller lambda0 \
	                                    than lambda1 and lambda2, particularly when data is noisy. Otherwise, \
	                                    lambda2 can be set a larger value than lambda0 and lambda1 produces a \
	                                    smoother result (optional).");
        inputTolerance_.setDescription("This positive floating point value specifies the tolerance of the \
	                                    iterative linear system solver. For noisy data, larger values in the range [1.0e-6 - 1.0e-7] \
                                        is needed to provide a smoother reconstruction.Values in the range [1.0e-7 - 1.0e-9] \
	                                    can be used.");
        inputSamplesPerNode_.setDescription("This positive integer value specifies the minimum number of sample points that should fall \
                                                within an octree node. For noisy data, larger values in the range [5.0 - 10.0] is needed \
                                                to provide a smoother reconstruction. Otherwise, small values in the range [1.0 - 5.0] can be used.");


    }
    
    /**
     *
     */
    bool SmootheSignedDistanceImpl::execute()
    {
        CSIRO::Mesh::MeshModelInterface& pointCloud        = *dataPointCloud_;
        bool&                            computeColor      = *dataComputeColor_;
        double&                          bboxExpFactor     = *dataBboxExpFactor_;
        int&                             octreeLevels      = *dataOctreeLevels_;
        int&                             octreeStartLevel  = *dataOctreeStartLevel_;
        int&                             samplesPerNode    = *dataSamplesPerNode_;
        double&                          isolevel          = *dataIsolevel_;
        CSIRO::Mesh::Vector3dGroup&      weight            = *dataWeight_;
        double&                          colorWeight0      = *dataColorWeight0_;
        double&                          colorWeight1      = *dataColorWeight1_;
        double&                          tolerance         = *dataTolerance_;
        double&                          colorTolerance    = *dataColorTolerance_;
        int&                             minimumIterations = *dataMinimumIterations_;
        int&                             maximumIterations = *dataMaximumIterations_;
        bool&                            fastConstruction  = *dataFastConstruction_;
        bool&                            makePolygons      = *dataMakePolygons_;
        CSIRO::Mesh::MeshModelInterface& outputMesh        = *dataOutputMesh_;
        
        outputMesh.clear();
        //Conver to Vector3d from group (group is easier to assign in WS)
        Mesh::Vector3d w = weight;
        //Parse settings
        Settings D;
        D.manifold = *dataManifold_;
        D.debug = *dataVerbose_;
        D.computeColor = computeColor;
        D.bboxExpansionFactor = bboxExpFactor;
        D.octreeLevels = octreeLevels;
        D.octreeLevelStart = octreeStartLevel;
        D.samplesPerNode = samplesPerNode;
        D.isoLevel = isolevel;
        D.weight0 = w.x;
        D.weight1 = w.y;
        D.weight2 = w.z;
        D.weight0_color = colorWeight0;
        D.weight1_color = colorWeight1;
        D.solverTolerance = tolerance;
        D.solverTolerance_color = colorTolerance;
        D.minIterations = minimumIterations;
        D.maxIterations = maximumIterations;
        D.fast = fastConstruction;
        D.polygons = makePolygons;
       

        //For test cases where nx, ny, nz are states
        Mesh::MeshNodesInterface& nodes = pointCloud.getNodes();
        if (nodes.hasState("nx") && nodes.hasState("ny") && nodes.hasState("nz"))
        {
        const Mesh::NodeStateHandle& nxHandle = nodes.getStateHandle("nx");
        const Mesh::NodeStateHandle& nyHandle = nodes.getStateHandle("ny");
        const Mesh::NodeStateHandle& nzHandle = nodes.getStateHandle("nz");
        const Mesh::NodeStateHandle& normHandle = nodes.addState<Mesh::Vector3D>("normal",Mesh::Vector3d());
        for (Mesh::MeshNodesInterface::const_iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
        {
            double normx,normy,normz;
            nodes.getState(*iter,nxHandle,normx);
            nodes.getState(*iter,nyHandle,normy);
            nodes.getState(*iter,nzHandle,normz);
            nodes.setState(*iter,normHandle,Mesh::Vector3d(normx,normy,normz));
        }
        }


        //run SSD
        if (D.computeColor)
        {
            OctreeBundle<SSDColorType> bundle(D);
            if (!PointReader::load_fromMMI(pointCloud, bundle.points))
            {
                std::cout << QString("ERROR: Cannot convert meshmodel interface to SSD format") + "\n";
                return false;
            }
            std::vector<typename SSDColorType::OutPointType> vertices;
            std::vector<std::vector<size_t> > faces;
            run_ssd(D,bundle,vertices,faces);
            PlyWriter::write_toMMI(outputMesh,vertices,faces,computeColor);

        }
        else
        {
            OctreeBundle<SSDType> bundle(D);
            if (!PointReader::load_fromMMI(pointCloud, bundle.points))
            {
                std::cout << QString("ERROR: Cannot convert meshmodel interface to SSD format") + "\n";
                return false;
            }
            std::vector<typename SSDType::OutPointType> vertices;
            std::vector<std::vector<size_t> > faces;
            run_ssd(D,bundle,vertices,faces);
            PlyWriter::write_toMMI(outputMesh,vertices,faces,computeColor);
        }
        //typename PointContainer::value_type p;
        // If your operation always succeeds, leave the following line
        // returning true. Otherwise, add your own logic to determine
        // whether or not the operation succeeded and return true only
        // if no error was encountered.
        return true;
    }


    /**
     *
     */
    SmootheSignedDistance::SmootheSignedDistance() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< SmootheSignedDistance >::getInstance(),
            tr("SSD reconstruction"))
    {
        pImpl_ = new SmootheSignedDistanceImpl(*this);
    }


    /**
     *
     */
    SmootheSignedDistance::~SmootheSignedDistance()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  SmootheSignedDistance::execute()
    {
        return pImpl_->execute();
    }
}}


using namespace CSIRO::PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(SmootheSignedDistance, 
                                   CSIRO::PointCloud::PointCloudPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PointCloud"))

