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
#include "Mesh/DataStructures/MeshModelInterface/meshelementsinterface.h"

#include "pclmeshnodesinterface.h"

namespace CSIRO
{

namespace PointCloud
{
    /**
     * \brief  A Workspace MeshModelInterface wrapper around a PCL cloud
     */
    template<typename P>
    class TypedPclMeshModelInterface : public Mesh::TypedMeshModelInterface<TypedPclMeshModelInterface<P> >
    {
        PclMeshNodesInterface<P> nodes_;

    public:
        typedef P                   point_t;
        typedef pcl::PointCloud<P>  pointcloud_t;

        TypedPclMeshModelInterface();
        TypedPclMeshModelInterface(const TypedPclMeshModelInterface& other);

        const TypedPclMeshModelInterface& operator=(const TypedPclMeshModelInterface& other);

        const DataExecution::DataFactory& getPointType() const;

        virtual ~TypedPclMeshModelInterface();

        virtual TypedPclMeshModelInterface*          clone() const;

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

        typename PclMeshNodesInterface<P>::pointcloud_t::Ptr getPointCloud();

    protected:
        virtual bool                            fastAssignFrom(const Mesh::MeshModelInterface& other);
    };


    typedef TypedPclMeshModelInterface<pcl::PointXYZRGBNormal> PclMeshModelInterface;
    typedef TypedPclMeshModelInterface<pcl::PointXYZRGBNormal> PclMeshModelInterfaceXYZRGBNormal;
    typedef TypedPclMeshModelInterface<pcl::PointXYZ>          PclMeshModelInterfaceXYZ;



    /**
     *
     */
    template<typename P>
    TypedPclMeshModelInterface<P>::TypedPclMeshModelInterface()
    {
    }


    /**
     *
     */
    template<typename P>
    TypedPclMeshModelInterface<P>::TypedPclMeshModelInterface(const TypedPclMeshModelInterface<P>& other) :
        nodes_(other.nodes_)
    {
    }


    /**
     *
     */
    template<typename P>
    const TypedPclMeshModelInterface<P>& TypedPclMeshModelInterface<P>::operator=( const TypedPclMeshModelInterface<P>& other )
    {
        if (&other != this)
        {
            nodes_ = other.nodes_;
        }

        return *this;
    }


    /**
     * \return The data type associated with the point struct used to store point data in this
     *         MeshModelInterface implementation.
     */
    template<typename P>
    const DataExecution::DataFactory& TypedPclMeshModelInterface<P>::getPointType() const
    {
        return DataExecution::DataFactoryTraits<P>::getInstance();
    }


    /**
     *
     */
    template<typename P>
    bool TypedPclMeshModelInterface<P>::fastAssignFrom( const Mesh::MeshModelInterface& other )
    {
        if (&other.getFactory() == &this->getFactory())
        {
            *this = static_cast<const TypedPclMeshModelInterface<P>&>(other);
            return true;
        }

        return false;
    }


    /**
     *
     */
    template<typename P>
    TypedPclMeshModelInterface<P>::~TypedPclMeshModelInterface()
    {
    }


    /**
     *
     */
    template<typename P>
    TypedPclMeshModelInterface<P>* TypedPclMeshModelInterface<P>::clone() const
    {
        return new TypedPclMeshModelInterface(*this);
    }


    /**
     *
     */
    template<typename P>
    Mesh::MeshNodesInterface& TypedPclMeshModelInterface<P>::getNodes()
    {
        return nodes_;
    }


    /**
     *
     */
    template<typename P>
    const Mesh::MeshNodesInterface& TypedPclMeshModelInterface<P>::getNodes() const
    {
        return nodes_;
    }


    /**
     *
     */
    template<typename P>
    Mesh::MeshElementsInterface& TypedPclMeshModelInterface<P>::getElements(const Mesh::ElementType::Type& type)
    {
        return Mesh::MeshElementsInterface::getNullMeshElementsInterface();
    }


    /**
     *
     */
    template<typename P>
    const Mesh::MeshElementsInterface& TypedPclMeshModelInterface<P>::getElements(const Mesh::ElementType::Type& type) const
    {
        return Mesh::MeshElementsInterface::getNullMeshElementsInterface();
    }


    /**
     *
     */
    template<typename P>
    void TypedPclMeshModelInterface<P>::emptyTrash()
    {
    }


    /**
     *
     */
    template<typename P>
    bool TypedPclMeshModelInterface<P>::isElementTypeSupported( const Mesh::ElementType::Type& type ) const
    {
        return false;
    }


    /**
     *
     */
    template<typename P>
    Mesh::ElementType::List TypedPclMeshModelInterface<P>::getElementTypesPresentInModel() const
    {
        return Mesh::ElementType::List();
    }


    /**
     *
     */
    template<typename P>
    Mesh::ElementType::List TypedPclMeshModelInterface<P>::getElementTypesWithStatesDefined() const
    {
        return Mesh::ElementType::List();
    }


    /**
     *
     */
    template<typename P>
    bool TypedPclMeshModelInterface<P>::generateAttachmentInfo( const Mesh::ElementType::Type& elementType )
    {
        return true;
    }


    /**
     *
     */
    template<typename P>
    void TypedPclMeshModelInterface<P>::discardAttachmentInfo( const Mesh::ElementType::Type& elementType )
    {
    }


    /**
     *
     */
    template<typename P>
    bool TypedPclMeshModelInterface<P>::attachmentInfoExistsFor( const Mesh::ElementType::Type& elementType ) const
    {
        return false;
    }


    /**
     *
     */
    template<typename P>
    Mesh::ElementType::List TypedPclMeshModelInterface<P>::attachmentInfoExistsFor() const
    {
        return Mesh::ElementType::List();
    }


    /**
     *
     */
    template<typename P>
    Mesh::ElementHandleList TypedPclMeshModelInterface<P>::getAttachedElements(
        const Mesh::ElementType::Type& elementType,
        const Mesh::NodeHandle& nodeHandle ) const
    {
        assert(false);
        return Mesh::ElementHandleList();
    }


    /**
     * \return
     */
    template<typename P>
    typename PclMeshNodesInterface<P>::pointcloud_t::Ptr TypedPclMeshModelInterface<P>::getPointCloud()
    {
        return nodes_.getPointCloud();
    }


    /**
     * \return The underlying point cloud if \a mesh is of type PclMeshNodesInterface, otherwise creates one
     */
    template<typename P>
    typename PclMeshNodesInterface<P>::pointcloud_t::Ptr createPointCloud( Mesh::MeshModelInterface& mesh )
    {
        if (&mesh.getFactory() == &DataExecution::DataFactoryTraits<TypedPclMeshModelInterface<P> >::getInstance())
        {
            return static_cast<TypedPclMeshModelInterface<P>&>(mesh).template getPointCloud<P>();
        }
    }


}}   // end of namespaces

DECLARE_WORKSPACE_DATA_FACTORY(CSIRO::PointCloud::PclMeshModelInterface, CSIRO_POINTCLOUD_API)
DECLARE_WORKSPACE_DATA_FACTORY(CSIRO::PointCloud::PclMeshModelInterfaceXYZ, CSIRO_POINTCLOUD_API)
DECLARE_WORKSPACE_DATA_FACTORY(pcl::PointXYZ, CSIRO_POINTCLOUD_API)
DECLARE_WORKSPACE_DATA_FACTORY(pcl::PointXYZRGBNormal, CSIRO_POINTCLOUD_API)

#endif
