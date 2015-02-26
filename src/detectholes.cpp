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

#include "pointcloudplugin.h"
#include "detectholes.h"

namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{

    /**
     * \internal
     */
    class DetectHolesImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(DetectHolesImpl)

    public:
        DetectHoles&  op_;

        // Data objects
        DataExecution::TypedObject< MeshModelInterface >  dataMesh_;

        // Inputs and outputs
        DataExecution::InputScalar inputMesh_;
    DataExecution::Output      outputMesh_;

        DetectHolesImpl(DetectHoles& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    DetectHolesImpl::DetectHolesImpl(DetectHoles& op) :
        op_(op),
        dataMesh_(),
        inputMesh_("Input Mesh", dataMesh_, op_),
        outputMesh_("Mesh Model", dataMesh_, op_)
    {
    }
        

    /**
     *
     */
    bool DetectHolesImpl::execute()
    {
        
        MeshModelInterface& mesh = *dataMesh_;
        MeshNodesInterface& nodes = mesh.getNodes();
        MeshElementsInterface& elements = mesh.getElements(ElementType::Tri::getInstance());
        //Should add a check for tri's as well - getElementTypeList
        if (elements.empty())
        {
            std::cout << QString("ERROR: Model does not contain any elements") + "\n";
            return false;
        }

    
        //Add states for free edge and length
        const NodeStateHandle& isFreeEdge = nodes.addState<MeshModelInterface::int_type>("freeedge",-1 );//Free edge detection (-1 is not checked, 0 is not a free edge, 1 is a free edge)
        const NodeStateHandle& edgeLength = nodes.addState<MeshModelInterface::int_type>("Edge element length",0);
        //Generate attachment info
        if (!mesh.generateAttachmentInfo(ElementType::Tri::getInstance()))
        {
            std::cout << QString("ERROR: Could not generate attachment info for mesh") + "\n";
            return false;
        }
        //Loop through the nodes
        MeshNodesInterface::iterator nIter = nodes.begin();
        MeshNodesInterface::iterator end = nodes.end();
        int freeCount;
        int nodeFreeState;
        int numHoles = 0; 
        int nonManifold = 0;
        int endOfList = 0;
        for(; nIter != end; ++nIter)
        {
            nodes.getState(*nIter,isFreeEdge,nodeFreeState);
            if (nodeFreeState == -1)//Node has not been checked
            {
                if (mesh.getAttachedNodes(ElementType::Tri::getInstance(),*nIter).size() != mesh.getAttachedElements(ElementType::Tri::getInstance(),*nIter).size())//Must be a free edge
                {
                    numHoles += 1;
                    nodes.setState(*nIter,isFreeEdge,1);//Beginning of 'hole'
                    NodeHandle holdingnode = *nIter;
                    //std::cout << QString("Found a free edge node, id is %1").arg(holdingnode.getIndex()) + "\n";
                    std::vector<NodeHandle> holeVector;
                    holeVector.push_back(*nIter);
                    freeCount = 1; //Set the element count
                    NodeHandleList nodeList = mesh.getAttachedNodes(ElementType::Tri::getInstance(),*nIter);
                    NodeHandleList::iterator subnIter = nodeList.begin();
                    //std::cout << QString("It has %1 connected nodes").arg(nodeList.size()) + "\n";
                    if (nodeList.size() > 2)
                    {
                        while (*subnIter != *nIter)
                        {
                            nodes.getState(*subnIter,isFreeEdge,nodeFreeState);//Get the nodes free edge state
                            //Case: Is not checked (node free state = -1) and is a free edge
                            if (nodeFreeState == -1 && mesh.getAttachedNodes(ElementType::Tri::getInstance(),*subnIter).size() != mesh.getAttachedElements(ElementType::Tri::getInstance(),*subnIter).size())
                            {
                                NodeHandle n = *subnIter;
                                //std::cout << QString("Found an unchecked free node, id is %1 ").arg(n.getIndex()) + "\n";
                                //Now check that it would be travelling along a free edge
                                ElementHandleList elemList = mesh.getAttachedElements(ElementType::Tri::getInstance(),*subnIter);
                                ElementHandleList prevElemList = mesh.getAttachedElements(ElementType::Tri::getInstance(),holdingnode);
                                int joinedElems = 0;//If there is only 1 it is a free edge
                                for (ElementHandleList::iterator eIt = elemList.begin(); eIt != elemList.end(); ++eIt)
                                {
                                    for (ElementHandleList::iterator prevEIt = prevElemList.begin(); prevEIt != prevElemList.end(); ++prevEIt)
                                    {
                                        if (*eIt == *prevEIt)
                                        {
                                            joinedElems = joinedElems + 1;
                                        }
                                    }
                                }
                                //std::cout << QString("It has %1 shared elements with the previous node").arg(joinedElems) + "\n";
                                if (joinedElems == 2)//This edge is not a free edge,manifold geometry
                                {
                                    //  std::cout << QString("Node is not along a free edge, skipping") + "\n";
                                    ++subnIter;
                                }
                                else if (joinedElems > 2)
                                {
                                    ++nonManifold;
                                    break;
                                }
                                else 
                                {
                                    freeCount = freeCount + 1;//Increment the free edge element count
                                    holeVector.push_back(*subnIter);//Add the node to the vector of holes
                                    nodes.setState(*subnIter,isFreeEdge,1);//Give it a free edge Id
                                    holdingnode = *subnIter;
                                    nodeList = mesh.getAttachedNodes(ElementType::Tri::getInstance(),*subnIter);//Get the new node connectionss
                                    //std::cout << QString("It has %1 connected nodes").arg(nodeList.size()) + "\n";
                                    subnIter = nodeList.begin();//Initialise the iterator
                                }
                            }
                            //Case: Is not checked (node free state = -1) and is not a free edge
                            else if (nodeFreeState == -1 && mesh.getAttachedNodes(ElementType::Tri::getInstance(),*subnIter).size() == mesh.getAttachedElements(ElementType::Tri::getInstance(),*subnIter).size())
                            {
                                nodes.setState(*subnIter,isFreeEdge,0);
                                ++subnIter;
                            }
                            //Case: Checked node - either free (must be a previous node) or not free
                            else
                            {
                                ++subnIter;
                            }
                            if (subnIter == nodeList.end())
                            {
                                ++endOfList;
                                break;
                                //return false;
                            }
                        }//End while loop
                        //std::cout << QString("Closed a hole, length was %1").arg(freeCount) + "\n";
                        freeCount = freeCount + 1;//Add last element to complete loop
                        //Now loop through hole vector, adding the number of nodes in the 'hole'
                        for (std::vector<NodeHandle>::const_iterator vIter = holeVector.begin(); vIter != holeVector.end(); ++vIter)
                        {
                            nodes.setState(*vIter, edgeLength, freeCount);
                        }
                        holeVector.clear();//Clear the hole vector
                    }//End if loop for non isolated tris
                    else 
                    {
                        //std::cout << QString("Warning, found an isolated triangle") + "\n";
                    }
                }//
                else
                {   
                    nodes.setState(*nIter,isFreeEdge,0);
                }
            }
        }
        std::cout << QString("Found %1 non-manifold tris, %2 end of list elements").arg(nonManifold).arg(endOfList) + "\n";
        std::cout << QString("Hole detection finished, found %1 free edge loops").arg(numHoles) + "\n";
        return true;
    }


    /**
     *
     */
    DetectHoles::DetectHoles() :
        DataExecution::Operation(
            DataExecution::OperationFactoryTraits< DetectHoles >::getInstance(),
            tr("Detect holes in mesh"))
    {
        pImpl_ = new DetectHolesImpl(*this);
    }


    /**
     *
     */
    DetectHoles::~DetectHoles()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  DetectHoles::execute()
    {
        return pImpl_->execute();
    }
}
}


using namespace CSIRO;
using namespace PointCloud;
DEFINE_WORKSPACE_OPERATION_FACTORY(DetectHoles, 
                                   PointCloud::PointCloudPlugin::getInstance(),
                                   DataExecution::Operation::tr("PointCloud"))
