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
#include "unorganizedsurfacereconstruction.h"

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{

    /**
     * \internal
     */
    class UnorganizedSurfaceReconstructionImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(UnorganizedSurfaceReconstructionImpl)

    public:
        UnorganizedSurfaceReconstruction&  op_;

        // Data objects
        DataExecution::TypedObject< MeshModelInterface >  dataMesh_;
        DataExecution::TypedObject< MeshModelInterface >  dataOutputMesh_;
        DataExecution::TypedObject< double >              searchRadius_;
        DataExecution::TypedObject< double >              multiplier_;
        DataExecution::TypedObject< double >              maximumNearestNeighbour_;
        DataExecution::TypedObject< double >              maximumSurfaceAngle_;
        DataExecution::TypedObject< double >              minimumAngle_;
        DataExecution::TypedObject< double >              maximumAngle_;
        DataExecution::TypedObject< bool >                quickReconstruct_;
        DataExecution::TypedObject< bool >                normalConsist_;
        DataExecution::TypedObject< bool >                vertexOrdering_;
        DataExecution::TypedObject< QString >             normalName_;
        DataExecution::TypedObject< QString >             curvName_;



        // Inputs and outputs
        DataExecution::InputScalar inputMesh_;
        DataExecution::InputScalar inputquickReconstruct_;
        DataExecution::InputScalar inputnormalConsist_;
        DataExecution::InputScalar inputvertexOrdering_;
        DataExecution::InputScalar inputSearchRadius_;
        DataExecution::InputScalar inputMultiplier_;
        DataExecution::InputScalar inputmaximumNearestNeighbour_;
        DataExecution::InputScalar inputmaximumSurfaceAngle_;
        DataExecution::InputScalar inputminimumAngle_;
        DataExecution::InputScalar inputmaximumAngle_;
        DataExecution::InputScalar inputnormalName_;
        DataExecution::InputScalar inputcurvName_;
        DataExecution::Output      outputMesh_;


        UnorganizedSurfaceReconstructionImpl(UnorganizedSurfaceReconstruction& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    UnorganizedSurfaceReconstructionImpl::UnorganizedSurfaceReconstructionImpl(UnorganizedSurfaceReconstruction& op) :
        op_(op),
        dataMesh_(),
        dataOutputMesh_(),
        searchRadius_(0.025),
        multiplier_(2.5),
        maximumNearestNeighbour_(100),
        maximumSurfaceAngle_(45.0),
        minimumAngle_(10.0),
        maximumAngle_(120.0),
        quickReconstruct_(true),
        normalConsist_(true),
        vertexOrdering_(true),
        normalName_("normal"),
        curvName_("curvature"),
        inputMesh_("Points", dataMesh_, op_),
        inputquickReconstruct_("Fast calculate normals", quickReconstruct_, op_),
        inputnormalConsist_("Normals are consistent", normalConsist_, op_),
        inputvertexOrdering_("Vertex ordering is consistent", quickReconstruct_, op_),
        inputSearchRadius_("Search Radius", searchRadius_, op_),
        inputMultiplier_("Multiplier", multiplier_, op_),
        inputmaximumNearestNeighbour_("Maximum Nearest Neighbour", maximumNearestNeighbour_, op_),
        inputmaximumSurfaceAngle_("Maximum Surface Angle", maximumSurfaceAngle_, op_),
        inputminimumAngle_("Minimum Angle", minimumAngle_, op_),
        inputmaximumAngle_("Maximum Angle", maximumAngle_, op_),
        inputnormalName_("Normal state name", normalName_, op_),
        inputcurvName_("Curvature state name", curvName_, op_),
        outputMesh_("Mesh Model", dataOutputMesh_, op_)
    {
        inputquickReconstruct_.setDescription("Enables fast compuation of normals and curvature, must be used if normals and curvature have not been determined.\
                                              However usually results in a very rough mesh, only use if your pointcloud is relatively smooth and regular.");
        inputnormalConsist_.setDescription("Sets the flag if normals are oriented consistently (i.e. all oriented to the viewpoint)");
        inputvertexOrdering_.setDescription("Sets the flag to order the resulting triangle vertices consistently (positive direction around normal).\
                                            Has to assume consistently oriented normals.");
        inputSearchRadius_.setDescription("Sphere radius for determining the k-nearest neighbors for triangulation.");
        inputMultiplier_.setDescription("Set the multiplier of the nearest neighbor distance to obtain the final search radius for each point\
                                        This makes the algorithim adapt to different point densities.");
        inputmaximumNearestNeighbour_.setDescription("Sets the maximum number of nearest neighbors to be searched for.");
        inputmaximumSurfaceAngle_.setDescription("Do not consider point for triangulation if their normal deviates more than this value from the query point normal.\
                                                 This ensures correct triangulation by avoidign connecting poitns from one side to points from the other through forcing\
                                                 the use of edge points.");
        inputminimumAngle_.setDescription("Set the minimum angle each triangle should have. As this is a greedy approach, it may be violated.");
        inputmaximumAngle_.setDescription("Sets the maximum angle each triangle can have. For best results, this value should be around 120 degrees.");
    }


    /**
     *
     */
    bool UnorganizedSurfaceReconstructionImpl::execute()
    {
        MeshModelInterface& mesh = *dataMesh_;
        MeshModelInterface& outputMesh = *dataOutputMesh_;

        MeshNodesInterface& nodes = mesh.getNodes();
        outputMesh.clear();
        MeshNodesInterface& outputNodes = outputMesh.getNodes();
        const NodeStateHandle* pixIdStateIn = 0;
        const NodeStateHandle* textureSStateOut = 0;
        const NodeStateHandle* textureTStateOut = 0;
        if (nodes.hasState("pixId"))
        {
            pixIdStateIn = &nodes.getStateHandle("pixId");
            textureSStateOut = &outputNodes.addState<double>("textureS", 0.0);
            textureTStateOut = &outputNodes.addState<double>("textureT", 0.0);
        }

        const NodeStateHandle& normalStateOut = outputNodes.addState("normal", Vector3d(0,0,-1));

        if(nodes.size() == 0)
        {
            return true;
        }

        // Output has the PointNormal type in order to store the normals
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    
        if (!*quickReconstruct_ && !nodes.hasState(*normalName_) && !nodes.hasState(*curvName_))
        {
            std::cout << QString("ERROR: Normal calculation is off, but model does not have states named %1 and %2. Ensure normals and curvatures are pre-calculated or set normal calculation to TRUE").arg(*normalName_).arg(*curvName_) + "\n";
            return false;
        }   

        if(*quickReconstruct_)
        {
            //For each node add point to point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            MeshNodesInterface::const_iterator nIter = nodes.begin();
            MeshNodesInterface::const_iterator end = nodes.end();
            for(; nIter != end; ++nIter)
            {
                Vector3d v = nodes.getPosition(*nIter);
                cloud->push_back(pcl::PointXYZ(v.x,v.y,v.z));
            }

            // Create a KD-Tree
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

            // Normal estimation*
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            tree->setInputCloud(cloud);
            n.setInputCloud(cloud);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.compute(*normals);
            //* normals should now contain the point normals + surface curvatures

            // Concatenate the XYZ and normal fields*
            pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        }
        else
        {
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
        }
    
        if (cloud_with_normals->size() < 3)
        {
            std::cout << QString("ERROR: Processed point cloud is too small, has %1 points.").arg(cloud_with_normals->size()) + "\n";
            return false;
        }
        //Sets KdTree for reconstruction
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(*searchRadius_);

        double deg2Rad = M_PI / 180;

        // Set typical values for the parameters
        gp3.setMu(*multiplier_);
        gp3.setMaximumNearestNeighbors(*maximumNearestNeighbour_);
        gp3.setMaximumSurfaceAngle(*maximumSurfaceAngle_ * deg2Rad);
        gp3.setMinimumAngle(*minimumAngle_ * deg2Rad);
        gp3.setMaximumAngle(*maximumAngle_ * deg2Rad);
        gp3.setNormalConsistency(*normalConsist_);
        gp3.setConsistentVertexOrdering(*vertexOrdering_);

        // Get result
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

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
        size_t numPointsInCloud = cloud_with_normals->size();
        size_t numTris = triangles.polygons.size();
        
        QVector<bool> nodeHasTri(static_cast<int>(numPointsInCloud), false);
        for(int i = 0; i < numTris; ++i)
        {
            std::vector<uint32_t>& vertices = triangles.polygons[i].vertices;
            nodeHasTri[vertices[0]] = true;
            nodeHasTri[vertices[1]] = true;
            nodeHasTri[vertices[2]] = true;
        }

        //Adds vertices to output mesh model
        //Required as MLS method of reconstruction may change vertex position
        QVector<NodeHandle> nodeLookup(numPointsInCloud);
        MeshNodesInterface::const_iterator nIter = nodes.begin();
        for(int i = 0; i < numPointsInCloud; ++i)
        {
            if (nodeHasTri[i])
            {
                pcl::PointNormal& p = cloud_with_normals->at(i);
                NodeHandle nhnd = outputNodes.add(Vector3d(p.x,p.y,p.z));
                nodeLookup[i] = nhnd;

                outputNodes.setState(nhnd, normalStateOut, Vector3d(p.normal_x, p.normal_y, p.normal_z).unitVector());

                // copy all the states for this node
                for (int iState=0; iState<numNodeStates; ++iState)
                {
                    bool ok = nodes.getState(*nIter, *srcNodeStates[iState], *nodeStateData[iState]);
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
            ++nIter;
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
            std::vector<uint32_t>& vertices = triangles.polygons[i].vertices;
            tris.add(nodeLookup[vertices[0]],nodeLookup[vertices[1]],nodeLookup[vertices[2]]);
        }

        return true;
    }


    /**
     *
     */
    UnorganizedSurfaceReconstruction::UnorganizedSurfaceReconstruction() :
        DataExecution::Operation(
            DataExecution::OperationFactoryTraits< UnorganizedSurfaceReconstruction >::getInstance(),
            tr("Reconstruct Surface (Unorganized)"))
    {
        pImpl_ = new UnorganizedSurfaceReconstructionImpl(*this);
    }


    /**
     *
     */
    UnorganizedSurfaceReconstruction::~UnorganizedSurfaceReconstruction()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  UnorganizedSurfaceReconstruction::execute()
    {
        return pImpl_->execute();
    }
}
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(UnorganizedSurfaceReconstruction, 
                                   PointCloud::PointCloudPlugin::getInstance(),
                                   DataExecution::Operation::tr("PointCloud"))
