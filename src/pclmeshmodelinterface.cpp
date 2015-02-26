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

#include "Workspace/DataExecution/DataObjects/typeddatafactory.h"
#include "Workspace/DataExecution/DataObjects/datafactorytraits.h"

#include "Mesh/DataStructures/MeshModelInterface/meshelementsinterface.h"

#include "pointcloudplugin.h"
#include "pclmeshnodesinterface.h"
#include "pclmeshmodelinterface.h"

namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{
    /**
     * 
     */
    PclMeshModelInterface::PclMeshModelInterface()
    {
    }


    /**
     * 
     */
    PclMeshModelInterface::PclMeshModelInterface( const PclMeshModelInterface& other ) :
        TypedMeshModelInterface<PclMeshModelInterface>(other),
        nodes_(other.nodes_)
    {
    }


    /**
     * 
     */    
    const PclMeshModelInterface& PclMeshModelInterface::operator=( const PclMeshModelInterface& other )
    {
        if (&other != this)
        {
            nodes_ = other.nodes_;
        }

        return *this;
    }

   
    /**
     * 
     */
    bool PclMeshModelInterface::fastAssignFrom( const MeshModelInterface& other )
    {
        if (&other.getFactory() == &getFactory())
        {
            *this = static_cast<const PclMeshModelInterface&>(other);
            return true;
        }

        return false;
    }


    /**
     * 
     */
    PclMeshModelInterface::~PclMeshModelInterface()
    {
    }


    /**
     * 
     */
    PclMeshModelInterface* PclMeshModelInterface::clone() const
    {
        return new PclMeshModelInterface(*this);
    }


    /**
     * 
     */
    MeshNodesInterface& PclMeshModelInterface::getNodes()
    {
        return nodes_;
    }


    /**
     * 
     */
    const MeshNodesInterface& PclMeshModelInterface::getNodes() const
    {
        return nodes_;
    }


    /**
     * 
     */
    MeshElementsInterface& PclMeshModelInterface::getElements(const ElementType::Type& type)
    {
        return MeshElementsInterface::getNullMeshElementsInterface();
    }


    /**
     * 
     */
    const MeshElementsInterface& PclMeshModelInterface::getElements(const ElementType::Type& type) const
    {
        return MeshElementsInterface::getNullMeshElementsInterface();
    }

    
    /**
     * 
     */
    void PclMeshModelInterface::emptyTrash()
    {
    }


    /**
     * 
     */    
    bool PclMeshModelInterface::isElementTypeSupported( const ElementType::Type& type ) const
    {
        return false;
    }


    /**
     * 
     */
    ElementType::List PclMeshModelInterface::getElementTypesPresentInModel() const
    {
        return ElementType::List();
    }


    /**
     * 
     */
    ElementType::List PclMeshModelInterface::getElementTypesWithStatesDefined() const
    {
        return ElementType::List();
    }


    /**
     *
     */
    bool PclMeshModelInterface::generateAttachmentInfo( const ElementType::Type& elementType )
    {
        return true;
    }


    /**
     * 
     */
    void PclMeshModelInterface::discardAttachmentInfo( const ElementType::Type& elementType )
    {
    }


    /**
     * 
     */
    bool PclMeshModelInterface::attachmentInfoExistsFor( const ElementType::Type& elementType ) const
    {
        return false;
    }


    /**
     * 
     */
    ElementType::List PclMeshModelInterface::attachmentInfoExistsFor() const
    {
        return ElementType::List();
    }


    /**
     * 
     */
    ElementHandleList PclMeshModelInterface::getAttachedElements( 
        const ElementType::Type& elementType, 
        const NodeHandle& nodeHandle ) const
    {
        assert(false);
        return ElementHandleList();
    }


    /**
     * 
     */
    PclMeshNodesInterface::pointcloud_t::Ptr PclMeshModelInterface::getPointCloud()
    {
        return nodes_.getPointCloud();
    }


    /**
     * \return The underlying point cloud if \a mesh is of type PclMeshNodesInterface, otherwise creates one
     */
    PclMeshNodesInterface::pointcloud_t::Ptr createPointCloud( MeshModelInterface& mesh )
    {
        if (&mesh.getFactory() == 
            &DataFactoryTraits<PclMeshModelInterface>::getInstance())
        {
            return static_cast<PclMeshModelInterface&>(mesh).getPointCloud();
        }
    }


}}   // end of namespaces

DEFINE_WORKSPACE_DATA_FACTORY(CSIRO::PointCloud::PclMeshModelInterface, 
                              CSIRO::PointCloud::PointCloudPlugin::getInstance())
