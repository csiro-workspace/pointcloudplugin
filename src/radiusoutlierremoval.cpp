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
#include "Workspace/QtVersionCompat/qscopedpointer.h"

#include "Workspace/Application/LanguageUtils/streamqstring.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"
#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterface.h"

#include "pointcloudplugin.h"
#include "radiusoutlierremoval.h"
#include "mergenearbynodes.h" //For octree stuff

namespace CSIRO
{

namespace PointCloud
{
    /**
     * \internal
     */
    using namespace Mesh;
    using namespace DataExecution;

    class RadiusOutlierRemovalImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::RadiusOutlierRemovalImpl)

    public:
        RadiusOutlierRemoval&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< CSIRO::Mesh::MeshModelInterface >  dataInModel_;
        CSIRO::DataExecution::TypedObject< double >                           dataSearchRadius_;
        CSIRO::DataExecution::TypedObject< int >                              dataMinimumNeighbors_;


        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputInModel_;
        CSIRO::DataExecution::InputScalar inputSearchRadius_;
        CSIRO::DataExecution::InputScalar inputMinimumNeighbors_;
        CSIRO::DataExecution::Output      outputOutModel_;

        typedef Octree<MergeNearbyNodes::Particle> octree_t;
        typedef OctreeVector3<double>              octreeVector_t;

        RadiusOutlierRemovalImpl(RadiusOutlierRemoval& op);

        int   testNoInRadius(const octree_t& tree, const octreeVector_t& pos, int minNeighbours);
        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    RadiusOutlierRemovalImpl::RadiusOutlierRemovalImpl(RadiusOutlierRemoval& op) :
        op_(op),
        dataInModel_(),
        dataSearchRadius_(1.0),
        dataMinimumNeighbors_(10),
        inputInModel_("Point Cloud", dataInModel_, op_, true),
        inputSearchRadius_("Search Radius", dataSearchRadius_, op_),
        inputMinimumNeighbors_("Minimum neighbors", dataMinimumNeighbors_, op_),
        outputOutModel_("Point Cloud", dataInModel_, op_)
    {
    }


    /**
     *
     */
    bool RadiusOutlierRemovalImpl::execute()
    {
        CSIRO::Mesh::MeshModelInterface& inModel          = *dataInModel_;
        double&                          searchRadius     = *dataSearchRadius_;
        int&                             minimumNeighbors = *dataMinimumNeighbors_;
        
        //Input
        MeshNodesInterface& inputNodes = inModel.getNodes();
        BoundingBox bb = inputNodes.calculateBoundingBox();
        
        
        QScopedPointer<octree_t> octree( new octree_t(
            bb.getMinimum(),
            bb.getMaximum(),
            1,//Radius to length depth
            inputNodes.size()));
        
        uint targetDepth = octree->calculateTargetDepthForRadius(0);
        std::cout << QString("Using target octree depth of %1 for particles").arg(targetDepth) + "\n";
                
        //Finish if input mesh is empty
        if(inputNodes.size() == 0)
        {
            return true;
        }
        
        uint pId = 0;
        const uint progressInterval = 1000;
        uint nextProgressUpdate = 0;
        uint numNodesIn = inputNodes.size();

        //Add points to octree
        MeshNodesInterface::const_iterator nIter = inputNodes.begin();
        MeshNodesInterface::const_iterator end = inputNodes.end();
        for(; nIter != end; ++nIter, ++pId)
        {
            octree->add(MergeNearbyNodes::Particle(pId, inputNodes.getPosition(*nIter)),targetDepth);
            if (++nextProgressUpdate == progressInterval)
            {
                    nextProgressUpdate = 0;
                    op_.setProgress(float(pId)/numNodesIn*33);
            }
        }

        octree->reportStats();

        //Now run through the octree
        nIter = inputNodes.begin();
       
        pId = 0;
        nextProgressUpdate = 0;
        for (; nIter != end; ++nIter, ++pId)
        {
            if (testNoInRadius(*octree, inputNodes.getPosition(*nIter), minimumNeighbors) < minimumNeighbors)
            {
                inputNodes.remove(*nIter);
            }
            if (++nextProgressUpdate == progressInterval)
            {
                nextProgressUpdate = 0;
                op_.setProgress(33 + (float(pId)/numNodesIn*33));
            }

        }
        return true;
    }

    int RadiusOutlierRemovalImpl::testNoInRadius(const octree_t& tree, const octreeVector_t& pos, int minNeighbours)
    {
        int nneighbours = 0;
        double& radius = *dataSearchRadius_;
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
            bool descendedToChild = false;
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
                        descendedToChild = true;
                        break;
                    }
                }
            }    

            // Need to descend to a child of one of the current children
            if (descendedToChild)
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
                    //Add no of neighbours
                    ++nneighbours;
                }
                if (nneighbours > minNeighbours)
                {
                    break;
                }
                // next item
                itemIndex = tree.items_[itemIndex].nextItem_;
            }
            if (nneighbours > minNeighbours)
            {
                break;
            }
            treeHistory[currentTreeDepth] = 0; // reset history flag at this depth
            --currentTreeDepth;
            nodeIndex = tree.nodes_[nodeIndex].parent_;
        }
        return nneighbours;
    }

    /**
     *
     */
    RadiusOutlierRemoval::RadiusOutlierRemoval() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< RadiusOutlierRemoval >::getInstance(),
            tr("Radius outlier removial"))
    {
        pImpl_ = new RadiusOutlierRemovalImpl(*this);
    }


    /**
     *
     */
    RadiusOutlierRemoval::~RadiusOutlierRemoval()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  RadiusOutlierRemoval::execute()
    {
        return pImpl_->execute();
    }
}}


using namespace CSIRO::PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(RadiusOutlierRemoval, 
                                   CSIRO::PointCloud::PointCloudPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("PointCloud"))

