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

#include <cassert>

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"
#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterface.h"
#include "Mesh/DataStructures/MeshModelInterface/meshelementsinterface.h"
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"

#include "pointcloudplugin.h"
#include "organizedsurfacereconstruction.h"

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>

namespace CSIRO
{

namespace PointCloud
{

    /**
     * \internal
     */
    class OrganizedSurfaceReconstructionImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::OrganizedsurfaceReconstructionImpl)

    public:
        OrganizedSurfaceReconstruction&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  dataMesh_;
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  dataOutputMesh_;


        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputMesh_;
        CSIRO::DataExecution::Output      outputMesh_;


        OrganizedSurfaceReconstructionImpl(OrganizedSurfaceReconstruction& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    OrganizedSurfaceReconstructionImpl::OrganizedSurfaceReconstructionImpl(OrganizedSurfaceReconstruction& op) :
        op_(op),
        dataMesh_(),
        dataOutputMesh_(),
        inputMesh_("PointCloud Points", dataMesh_, op_),
        outputMesh_("Mesh Model", dataOutputMesh_, op_)
    {
    }


    /**
     *
     */
    bool OrganizedSurfaceReconstructionImpl::execute()
    {
        return false; // todo:
#if 0
        CSIRO::Mesh::MeshModelInterface& mesh = *dataMesh_;
        CSIRO::Mesh::MeshModelInterface& outputMesh = *dataOutputMesh_;
        const int pixelNum = HardwareManager::SensorResolutionX*HardwareManager::SensorResolutionY;
        
        Mesh::MeshNodesInterface& nodes = mesh.getNodes();

        if(nodes.size() != pixelNum)
        {
            logText("Error points are not organized.\n");
            return false;
        }

        Mesh::MeshNodesInterface::const_iterator nIter = nodes.begin();
        Mesh::NodeHandle nodeLookup[pixelNum];

        //Declare Point Cloud and initiate size
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cloud->height = HardwareManager::SensorResolutionY;
        cloud->width = HardwareManager::SensorResolutionX;
        
        //Force point cloud to pixel size
        cloud->resize(pixelNum);

        outputMesh.clear();
        Mesh::MeshNodesInterface& outputNodes = outputMesh.getNodes();

        //For each pixel(node) add point to point cloud
        for(int i = 0; i < pixelNum; ++i)
        {
            Mesh::Vector3d v = nodes.getPosition(*nIter);

            nodeLookup[i] = (outputNodes.add(v));
            nIter++;

            cloud->at(i) = pcl::PointXYZ(v.x,v.y,v.z);
        }

        //Declare KdTree used in reconstruction method
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        //Declares mesh constructor object to reconstruct from point cloud data
        pcl::OrganizedFastMesh<pcl::PointXYZ> fastMesh;
        fastMesh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_LEFT_CUT);
        //Polygon Mesh stores calculated triangles in an array
        pcl::PolygonMesh triangles;
        //Set inputs for reconstruction
        fastMesh.setInputCloud (cloud);
        fastMesh.setSearchMethod (tree);
        fastMesh.reconstruct (triangles);

        Mesh::MeshElementsInterface& tris = outputMesh.getElements(Mesh::ElementType::Tri::getInstance());
        
        //For each triangle in the PolygonMesh add tri to mesh model
        for(int i = 0; i < triangles.polygons.size(); ++i)
        {
            //Accesses vertices array of each triangle
            std::vector<uint32_t>& vertices = triangles.polygons[i].vertices;
            tris.add(nodeLookup[vertices[0]],nodeLookup[vertices[1]],nodeLookup[vertices[2]]);
        }
        
        return true;
#endif
    }


    /**
     *
     */
    OrganizedSurfaceReconstruction::OrganizedSurfaceReconstruction() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< OrganizedSurfaceReconstruction >::getInstance(),
            tr("Reconstruct Surface (Organized)"))
    {
        pImpl_ = new OrganizedSurfaceReconstructionImpl(*this);
    }


    /**
     *
     */
    OrganizedSurfaceReconstruction::~OrganizedSurfaceReconstruction()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  OrganizedSurfaceReconstruction::execute()
    {
        return pImpl_->execute();
    }
}
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(OrganizedSurfaceReconstruction, 
                                   CSIRO::PointCloud::PointCloudPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PointCloud"))

