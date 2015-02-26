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
#include "Workspace/QtVersionCompat/qscopedpointer.h"

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"
#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterface.h"
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"

#include "pointcloudplugin.h"
#include "mergenearbynodes.h"


namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{
    /**
     * \internal
     */
    class MergeNearbyNodesImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::MergeNearbyNodesImpl)

    public:
        MergeNearbyNodes&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  meshIn_;
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  meshOut_;
        CSIRO::DataExecution::TypedObject< double >                           radius_;
        CSIRO::DataExecution::TypedObject< bool >                             dataConservativeMerge_;
        CSIRO::DataExecution::TypedObject< double >                           dataCurvatureLimit_;
        CSIRO::DataExecution::TypedObject< QString >                          dataCurvatureName_;

        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputMeshIn_;
        CSIRO::DataExecution::InputScalar inputRadius_;
        CSIRO::DataExecution::InputScalar inputConservativeMerge_;
        CSIRO::DataExecution::InputScalar inputCurvatureLimit_;
        CSIRO::DataExecution::InputScalar inputCurvatureName_;
        CSIRO::DataExecution::Output      outputMeshOut_;

        typedef Octree<MergeNearbyNodes::Particle> octree_t;
        typedef OctreeVector3<double>              octreeVector_t;

        MergeNearbyNodesImpl(MergeNearbyNodes& op);

        void  testAndMerge(const octree_t& tree, std::vector<uint>& mergeTo, uint pId, const octreeVector_t& pos);
        void  curvTestAndMerge(const octree_t& tree, std::vector<uint>& mergeTo, uint pId, const octreeVector_t& pos, double inCurvature);
        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    MergeNearbyNodesImpl::MergeNearbyNodesImpl(MergeNearbyNodes& op) :
        op_(op),
        radius_(1.0),
        dataConservativeMerge_(false),
        dataCurvatureLimit_(0.06),
        dataCurvatureName_("curvature"),
        inputMeshIn_("Mesh model", meshIn_, op_),
        inputRadius_("Radius", radius_, op_),
        inputConservativeMerge_("Conservative merging", dataConservativeMerge_, op_),
        inputCurvatureLimit_("Conservative curvature limit", dataCurvatureLimit_, op_),
        inputCurvatureName_("Curvature state name", dataCurvatureName_, op_),
        outputMeshOut_("Mesh model", meshOut_, op_)
    {
        inputConservativeMerge_.setDescription("Enables conservative merging based on a curvature limit");
        inputCurvatureLimit_.setDescription("Limit of curvature if conservative merging is on");
    }


    /**
     *
     */
    bool MergeNearbyNodesImpl::execute()
    {
        meshOut_->clear();

        const MeshNodesInterface& nodesIn = meshIn_->getNodes();
        BoundingBox bb = nodesIn.calculateBoundingBox();
        NodeStateHandle curvHandle;

        if (*dataConservativeMerge_)
        {
            if (!nodesIn.hasState(*dataCurvatureName_))
            {
                std::cout << QString("ERROR: Model does not have a state named %1").arg(*dataCurvatureName_) + "\n";
                return false;
            }
             curvHandle = nodesIn.getStateHandle(*dataCurvatureName_);
        }
        
        QScopedPointer<octree_t> octree(new octree_t(
            bb.getMinimum(),
            bb.getMaximum(),
            1,
            nodesIn.size()));

        uint targetDepth = octree->calculateTargetDepthForRadius(0);
        op_.logText(QString("Using target octree depth of %1 for particles\n").arg(targetDepth));

        uint pId = 0;
        const uint progressInterval = 10000;
        uint nextProgressUpdate = 0;
        uint numNodesIn = nodesIn.size();
        std::vector<uint> mergeTo(numNodesIn);
        MeshNodesInterface::const_iterator nIter = nodesIn.begin();
        MeshNodesInterface::const_iterator nEnd = nodesIn.end();
        for (; nIter != nEnd; ++nIter, ++pId)
        {
            octree->add(MergeNearbyNodes::Particle(pId, nodesIn.getPosition(*nIter)), targetDepth);
            mergeTo[pId] = pId;

            if (++nextProgressUpdate == progressInterval)
            {
                nextProgressUpdate = 0;
                op_.setProgress(float(pId)/numNodesIn*33);
            }
        }

        octree->reportStats();

        nIter = nodesIn.begin();
        pId = 0;
        nextProgressUpdate = 0;
        double curv;
        for (; nIter != nEnd; ++nIter, ++pId)
        {
            if (!*dataConservativeMerge_)
            {
                if (mergeTo[pId] == pId) // check node hasn't already been merged (SRM: And conservative is not on)
                    testAndMerge(*octree, mergeTo, pId, nodesIn.getPosition(*nIter));
            }
            else
            {
                nodesIn.getState(*nIter,curvHandle,curv);
                if (mergeTo[pId] == pId && curv < *dataCurvatureLimit_)//Check node hasn't been merged and has a low curvature
                   curvTestAndMerge(*octree, mergeTo, pId, nodesIn.getPosition(*nIter),curv);
            }
            if (++nextProgressUpdate == progressInterval)
            {
                nextProgressUpdate = 0;
                op_.setProgress(33 + (float(pId)/numNodesIn*33));
            }
        }

        MeshNodesInterface& nodesOut = meshOut_->getNodes();

        // Allocate DataObjects of the correct types to copy the state data with.
        // We also re-populate srcNodeStates to match dstNodeStates since this may
        // be less than than all the src states if allowPartialAssign is true. It's 
        // also possible the dst uses a different order.
        NodeStateHandleList srcNodeStates = nodesIn.getAllStateHandles();
        NodeStateHandleList dstNodeStates;
        QList<DataObject*> nodeStateData;
        foreach(const NodeStateHandle* state, srcNodeStates)
        {
            nodeStateData.push_back(state->getDataFactory().createDataObject());
            if (!nodesOut.hasState(state->getName()))
                nodesOut.addState(state->getName(), state->getDefaultValue());

            dstNodeStates.push_back(&nodesOut.getStateHandle(state->getName()));
            assert(dstNodeStates.back()->isValid());
        }

        int numNodeStates = dstNodeStates.size();
        nIter = nodesIn.begin();
        pId = 0;
        nextProgressUpdate = 0;
        for (; nIter != nEnd; ++nIter, ++pId)
        {
            if (mergeTo[pId] == pId)
            {
                NodeHandle n = nodesOut.add(nodesIn.getPosition(*nIter));

                // copy all the states for this node
                for (int iState=0; iState<numNodeStates; ++iState)
                {
                    bool ok = nodesIn.getState(*nIter, *srcNodeStates[iState], *nodeStateData[iState]);
                    assert(ok);
                    ok = nodesOut.setState(n, *dstNodeStates[iState], *nodeStateData[iState]);
                    assert(ok);
                }
            }

            if (++nextProgressUpdate == progressInterval)
            {
                nextProgressUpdate = 0;
                op_.setProgress(66 + (float(pId)/numNodesIn*33));
            }
        }

        // Delete the node state data objects used for copying states
        foreach(DataObject* dataObject, nodeStateData)
        {
            dataObject->getFactory().destroyDataObject(dataObject);
        }
        nodeStateData.clear();

        op_.logText(op_.tr("Merged mesh has %1% of original nodes (%2/%3)\n").arg(
            double(nodesOut.size())/nodesIn.size()).arg(
            nodesOut.size()).arg(
            nodesIn.size()));

        return true;
    }

    void MergeNearbyNodesImpl::testAndMerge(const octree_t& tree, std::vector<uint>& mergeTo, uint pId, const octreeVector_t& pos)
    {
        assert(mergeTo[pId] == pId); // done by calling code

        double& radius = *radius_;
        double radiusSq = radius * radius;

        // Have to use non-recursive tree traversal as recursion is not possible in OpenCL
        // http://www.nvidia.com/content/GTC/documents/1077_GTC09.pdf
        // https://developer.nvidia.com/content/thinking-parallel-part-ii-tree-traversal-gpu
        OctreeIndex nodeIndex = 0;
        int currentTreeDepth = 0;
        std::vector<unsigned> treeHistory(tree.maxDepth_+1, 0);    

        while (currentTreeDepth >= 0)
        {
            if (!octreeNodeIntersectsSphere(tree.nodes_[nodeIndex], pos, radius))
            {
                // Sphere does not intersect this octree node so step
                // back up the tree
                --currentTreeDepth;
                nodeIndex = tree.nodes_[nodeIndex].parent_;
                continue;
            }

            // Step into each valid child
            bool decendedToChild = false;
            for (int leafIndex=0; leafIndex<8; ++leafIndex)
            {
                if (!(treeHistory[currentTreeDepth] & (1<<leafIndex)))
                {
                    treeHistory[currentTreeDepth] |= (1<<leafIndex);
                    OctreeIndex childIndex = octreeNodeGetChild(tree.nodes_[nodeIndex], leafIndex);
                    if (childIndex != InvalidOctreeIndex)
                    {
                        // descend to child
                        nodeIndex = childIndex;
                        ++currentTreeDepth;
                        decendedToChild = true;
                        break;
                    }
                }
            }    

            // Need to descend to a child of one of the current children
            if (decendedToChild)
                continue;

            // If we got this far then all children have been visited so step back
            // up to parent after processing any items at this level
            OctreeIndex itemIndex = tree.nodes_[nodeIndex].items_;
            while (itemIndex != InvalidOctreeIndex)
            {
                // process item
                double distSq = (tree.items_[itemIndex].item_.getPosition() - pos).magnitudeSq();
                if (distSq <= radiusSq)
                {
                    // merge other node to me
                    mergeTo[tree.items_[itemIndex].item_.id_] = pId;
                }

                // next item
                itemIndex = tree.items_[itemIndex].nextItem_;
            }

            treeHistory[currentTreeDepth] = 0; // reset history flag at this depth
            --currentTreeDepth;
            nodeIndex = tree.nodes_[nodeIndex].parent_;
        }
    }
    void MergeNearbyNodesImpl::curvTestAndMerge(const octree_t& tree, std::vector<uint>& mergeTo, uint pId, const octreeVector_t& pos, double inCurvature)
    {
        assert(mergeTo[pId] == pId); // done by calling code

        double& radius = *radius_;
        double radiusSq = radius * radius;

        // Have to use non-recursive tree traversal as recursion is not possible in OpenCL
        // http://www.nvidia.com/content/GTC/documents/1077_GTC09.pdf
        // https://developer.nvidia.com/content/thinking-parallel-part-ii-tree-traversal-gpu
        OctreeIndex nodeIndex = 0;
        int currentTreeDepth = 0;
        std::vector<unsigned> treeHistory(tree.maxDepth_+1, 0);    

        while (currentTreeDepth >= 0)
        {
            if (!octreeNodeIntersectsSphere(tree.nodes_[nodeIndex], pos, radius))
            {
                // Sphere does not intersect this octree node so step
                // back up the tree
                --currentTreeDepth;
                nodeIndex = tree.nodes_[nodeIndex].parent_;
                continue;
            }

            // Step into each valid child
            bool decendedToChild = false;
            for (int leafIndex=0; leafIndex<8; ++leafIndex)
            {
                if (!(treeHistory[currentTreeDepth] & (1<<leafIndex)))
                {
                    treeHistory[currentTreeDepth] |= (1<<leafIndex);
                    OctreeIndex childIndex = octreeNodeGetChild(tree.nodes_[nodeIndex], leafIndex);
                    if (childIndex != InvalidOctreeIndex)
                    {
                        // descend to child
                        nodeIndex = childIndex;
                        ++currentTreeDepth;
                        decendedToChild = true;
                        break;
                    }
                }
            }    

            // Need to descend to a child of one of the current children
            if (decendedToChild)
                continue;

            // If we got this far then all children have been visited so step back
            // up to parent after processing any items at this level
            OctreeIndex itemIndex = tree.nodes_[nodeIndex].items_;
            while (itemIndex != InvalidOctreeIndex)
            {
                // process item
                double distSq = (tree.items_[itemIndex].item_.getPosition() - pos).magnitudeSq();
                if (distSq <= radiusSq && inCurvature < *dataCurvatureLimit_)
                {
                    // merge other node to me
                    mergeTo[tree.items_[itemIndex].item_.id_] = pId;
                }

                // next item
                itemIndex = tree.items_[itemIndex].nextItem_;
            }

            treeHistory[currentTreeDepth] = 0; // reset history flag at this depth
            --currentTreeDepth;
            nodeIndex = tree.nodes_[nodeIndex].parent_;
        }
    }

    /**
     *
     */
    MergeNearbyNodes::MergeNearbyNodes() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< MergeNearbyNodes >::getInstance(),
            tr("Merge Nearby Nodes"))
    {
        pImpl_ = new MergeNearbyNodesImpl(*this);
    }


    /**
     *
     */
    MergeNearbyNodes::~MergeNearbyNodes()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  MergeNearbyNodes::execute()
    {
        return pImpl_->execute();
    }
}
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(MergeNearbyNodes,
                                   CSIRO::PointCloud::PointCloudPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PointCloud"))

