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

/**
 * \file
 */

#ifndef CSIRO_POINTCLOUD_PCLMESHMODELINTERFACE_H
#define CSIRO_POINTCLOUD_PCLMESHMODELINTERFACE_H

#include "PointCloud/pointcloudplugin_api.h"
#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterface.h"

#include "pclmeshnodesinterface.h"

namespace CSIRO
{

namespace PointCloud
{
    /**
     * \brief  A Workspace MeshModelInterface wrapper around a PCL cloud
     */
    class CSIRO_POINTCLOUD_API PclMeshModelInterface : public Mesh::TypedMeshModelInterface<PclMeshModelInterface>
    {
        PclMeshNodesInterface nodes_;

    public:

        PclMeshModelInterface();
        PclMeshModelInterface(const PclMeshModelInterface& other);

        const PclMeshModelInterface& operator=(const PclMeshModelInterface& other);

        virtual ~PclMeshModelInterface();

        virtual PclMeshModelInterface*          clone() const;

        virtual Mesh::MeshNodesInterface&             getNodes();
        virtual const Mesh::MeshNodesInterface&       getNodes() const;

        virtual Mesh::MeshElementsInterface&          getElements(const Mesh::ElementType::Type& type);
        virtual const Mesh::MeshElementsInterface&    getElements(const Mesh::ElementType::Type& type) const; 

        virtual bool                            isElementTypeSupported(const Mesh::ElementType::Type& type) const;
        virtual Mesh::ElementType::List         getElementTypesPresentInModel() const;
        virtual Mesh::ElementType::List         getElementTypesWithStatesDefined() const;

        virtual void                            emptyTrash();

        virtual bool                            generateAttachmentInfo(const Mesh::ElementType::Type& elementType);
        virtual void                            discardAttachmentInfo(const Mesh::ElementType::Type& elementType);
        virtual bool                            attachmentInfoExistsFor(const Mesh::ElementType::Type& elementType) const;
        virtual Mesh::ElementType::List         attachmentInfoExistsFor() const;
        virtual Mesh::ElementHandleList         getAttachedElements(const Mesh::ElementType::Type& elementType, const Mesh::NodeHandle& node) const;

        PclMeshNodesInterface::pointcloud_t::Ptr getPointCloud();

    protected:

        virtual bool                            fastAssignFrom(const MeshModelInterface& other);
    };

    PclMeshNodesInterface::pointcloud_t::Ptr getOrCreatePointCloud(Mesh::MeshModelInterface& mesh);

}}   // end of namespaces

DECLARE_WORKSPACE_DATA_FACTORY(CSIRO::PointCloud::PclMeshModelInterface, CSIRO_POINTCLOUD_API)

#endif
