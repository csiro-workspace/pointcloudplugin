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

#ifndef CSIRO_MESH_PCLMESHNODESINTERFACE_H
#define CSIRO_MESH_PCLMESHNODESINTERFACE_H

#include <QException>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Mesh/Geometry/vector3d.h"
#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"

namespace CSIRO
{
 
namespace PointCloud
{

    /**
     * \brief A Workspace MeshNodesInterface wrapper around a PCL cloud
     */
    template<typename P>
    class PclMeshNodesInterface : public Mesh::MeshNodesInterface
    {
    public:
        PclMeshNodesInterface();
        PclMeshNodesInterface(const PclMeshNodesInterface&);
        const PclMeshNodesInterface& operator= (const PclMeshNodesInterface&);

        virtual Mesh::NodeHandle add(const Mesh::Vector3d& position);
        virtual Mesh::Vector3d   getPosition(const Mesh::NodeHandle& nodeHandle) const;
        virtual void             setPosition(const Mesh::NodeHandle& nodeHandle, const Mesh::Vector3d& position);

        virtual bool            isStateTypeSupported(const DataExecution::DataFactory& dataFactory) const;

        virtual size_type       size() const;  
        virtual void            clear();

        virtual iterator        erase(const iterator& position);
        virtual void            remove(const Mesh::NodeHandle& nodeHandle);

        virtual bool            empty() const;
        virtual void            reserve(size_type n);

        virtual bool            getState(const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, int_type& result) const;
        virtual bool            getState(const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, double& result) const;
        virtual bool            getState(const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, Mesh::Vector3d& result) const;

        virtual bool            setState(const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, int_type value);
        virtual bool            setState(const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, double value);
        virtual bool            setState(const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, const Mesh::Vector3d& value);

        virtual bool            setState(const Mesh::NodeHandle& nodeHandle, 
                                         const Mesh::NodeStateHandle& state, 
                                         const DataExecution::DataObject& value);

        virtual PclMeshNodesInterface* clone() const;

        typedef P                   point_t;
        typedef pcl::PointCloud<P>  pointcloud_t;
        typename pointcloud_t::Ptr getPointCloud() { return pointCloud_; }

    protected:

        virtual Mesh::NodeHandle advance(const Mesh::NodeHandle& nodeHandle, size_type n) const;

        virtual Mesh::NodeStateHandle* addStateToImplementation(const QString& name, const DataExecution::DataObject& defaultValue);

        virtual bool            removeStateFromImplementation(const Mesh::NodeStateHandle& state);

        virtual bool            getStateImpl(const Mesh::NodeHandle& nodeHandle, 
                                             const Mesh::NodeStateHandle& state, 
                                             DataExecution::DataObject& result);

        virtual bool            getStateImpl(const Mesh::NodeHandle& nodeHandle, 
                                             const Mesh::NodeStateHandle& state, 
                                             DataExecution::DataObject& result) const;

        virtual Mesh::NodeHandle beginHandle() const;
        virtual Mesh::NodeHandle endHandle() const;

    private:

        inline point_t& getPoint(const Mesh::NodeHandle& n)
        {
            return (*pointCloud_)[n.getIndex()];
        }

        inline const point_t& getPoint(const Mesh::NodeHandle& n) const
        {
            return (*pointCloud_)[n.getIndex()];
        }

        void initStates();

        typename pointcloud_t::Ptr   pointCloud_;
        const Mesh::NodeStateHandle* rgbaState_;
        const Mesh::NodeStateHandle* normalState_;
    };


    /**
     *
     */
    template<typename P>
    void PclMeshNodesInterface<P>::initStates()
    {
        registerExistingState(0, "RGBA", DataExecution::TypedObject<Mesh::MeshModelInterface::int_type>(0xFFFFFFFF));
        registerExistingState(1, "normal", DataExecution::TypedObject<Mesh::Vector3d>(Mesh::Vector3d(0,0,0)));

        rgbaState_ = &getStateHandle("RGBA");
        normalState_ = &getStateHandle("normal");
    }


    /**
     *
     */
    template<typename P>
    PclMeshNodesInterface<P>::PclMeshNodesInterface() :
        pointCloud_(new pointcloud_t()),
        rgbaState_(0),
        normalState_(0)
    {
        initStates();
    }


    /**
     *
     */
    template<typename P>
    PclMeshNodesInterface<P>::PclMeshNodesInterface( const PclMeshNodesInterface& other ) :
        MeshNodesInterface(other),
        pointCloud_(new pointcloud_t(*other.pointCloud_)),
        rgbaState_(0),
        normalState_(0)
    {
        initStates();
    }


    /**
     *
     */
    template<typename P>
    const PclMeshNodesInterface<P>& PclMeshNodesInterface<P>::operator=( const PclMeshNodesInterface<P>& other )
    {
        MeshNodesInterface::operator=(other);
        pointCloud_ = typename pointcloud_t::Ptr(new pointcloud_t(*other.pointCloud_));
        return *this;
    }


    /**
     *
     */
    template<typename P>
    Mesh::NodeHandle PclMeshNodesInterface<P>::add( const Mesh::Vector3d& position )
    {
        size_type newIndex = pointCloud_->size();
        point_t point;
        point.x = position.x;
        point.y = position.y;
        point.z = position.z;
        pointCloud_->push_back(point);
        return Mesh::NodeHandle(newIndex);
    }


    /**
     *
     */
    template<typename P>
    Mesh::Vector3d PclMeshNodesInterface<P>::getPosition( const Mesh::NodeHandle& nodeHandle ) const
    {
        assert(nodeHandle.getIndex() < static_cast<size_type>(pointCloud_->size()));
        const point_t& point = getPoint(nodeHandle);
        Mesh::Vector3d result(point.x, point.y, point.z);
        return result;
    }


    /**
     *
     */
    template<typename P>
    void PclMeshNodesInterface<P>::setPosition( const Mesh::NodeHandle& nodeHandle, const Mesh::Vector3d& position )
    {
        assert(nodeHandle.getIndex() < static_cast<size_type>(pointCloud_->size()));
        point_t& point = getPoint(nodeHandle);
        point.x = position.x;
        point.y = position.y;
        point.z = position.z;
    }


    /**
     *
     */
    template<typename P>
    bool PclMeshNodesInterface<P>::isStateTypeSupported( const DataExecution::DataFactory& dataFactory ) const
    {
        return true;
    }


    /**
     *
     */
    template<typename P>
    Mesh::MeshNodesInterface::size_type PclMeshNodesInterface<P>::size() const
    {
        return pointCloud_->size();
    }


    /**
     *
     */
    template<typename P>
    void PclMeshNodesInterface<P>::clear()
    {
        pointCloud_->clear();
    }


    /**
     *
     */
    template<typename P>
    typename PclMeshNodesInterface<P>::iterator PclMeshNodesInterface<P>::erase( const iterator& position )
    {
        pointCloud_->erase(pointCloud_->begin() + position->getIndex());
        return createIterator(Mesh::NodeHandle(position->getIndex() + 1));
    }


    /**
     *
     */
    template<typename P>
    void PclMeshNodesInterface<P>::remove( const Mesh::NodeHandle& nodeHandle )
    {
        pointCloud_->erase(pointCloud_->begin() + nodeHandle.getIndex());
    }


    /**
     *
     */
    template<typename P>
    bool PclMeshNodesInterface<P>::empty() const
    {
        return pointCloud_->empty();
    }


    /**
     *
     */
    template<typename P>
    void PclMeshNodesInterface<P>::reserve( size_type n )
    {
        pointCloud_->reserve(n);
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::setState(
        const Mesh::NodeHandle& nodeHandle,
        const Mesh::NodeStateHandle& state,
        const DataExecution::DataObject& value )
    {
        if (&value.getFactory() == &DataExecution::DataFactoryTraits<Mesh::MeshModelInterface::int_type>::getInstance())
        {
            setState(nodeHandle, state, value.getRawData<Mesh::MeshModelInterface::int_type>());
        }
        else if (&value.getFactory() == &DataExecution::DataFactoryTraits<Mesh::Vector3d>::getInstance())
        {
            setState(nodeHandle, state, value.getRawData<Mesh::Vector3d>());
        }
        else
        {
            return false;
        }

        return true;
    }


    /**
     *
     */
    template<>
    inline bool PclMeshNodesInterface<pcl::PointXYZRGBNormal>::setState(
        const Mesh::NodeHandle& nodeHandle,
        const Mesh::NodeStateHandle& state,
        const DataExecution::DataObject& value )
    {
        return false;
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::setState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, int_type value )
    {
        point_t point = getPoint(nodeHandle);
        Q_UNUSED(point)
        return false;
    }


    /**
     *
     */
    template<>
    inline bool PclMeshNodesInterface<pcl::PointXYZRGBNormal>::setState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, int_type value )
    {
        point_t point = getPoint(nodeHandle);
        if (&state == rgbaState_)
        {
            uint rgba = static_cast<uint>(value);
            point.r = (rgba) & 0xFF;
            point.g = (rgba >> 8) & 0xFF;
            point.b = (rgba >> 16) & 0xFF;
            point.a = (rgba >> 24) & 0xFF;
            return true;
        }

        return false;
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::setState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, double value )
    {
        return false;
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::setState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, const Mesh::Vector3d& value )
    {
        point_t& point = getPoint(nodeHandle);
        Q_UNUSED(point)
        return false;
    }


    /**
     *
     */
    template<>
    inline bool PclMeshNodesInterface<pcl::PointXYZRGBNormal>::setState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, const Mesh::Vector3d& value )
    {
        point_t& point = getPoint(nodeHandle);
        if (&state == normalState_)
        {
            point.normal_x = value.x;
            point.normal_y = value.y;
            point.normal_z = value.z;
            return true;
        }

        return false;
    }


    /**
     *
     */
    template<typename P>
    Mesh::NodeHandle PclMeshNodesInterface<P>::advance( const Mesh::NodeHandle& nodeHandle, size_type n ) const
    {
        return Mesh::NodeHandle(nodeHandle.getIndex() + n);
    }


    /**
     *
     */
    template<typename P>
    Mesh::NodeStateHandle* PclMeshNodesInterface<P>::addStateToImplementation( const QString& name, const DataExecution::DataObject& defaultValue )
    {
        return 0; // not supported
    }


    /**
     *
     */
    template<typename P>
    bool PclMeshNodesInterface<P>::removeStateFromImplementation( const Mesh::NodeStateHandle& state )
    {
        return false; // not supported
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::getStateImpl( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, DataExecution::DataObject& result )
    {
        // Use const version - base class that calls this will ensure data isn't shared which is the only difference b/w const and non-const version
        return static_cast<const PclMeshNodesInterface*>(this)->getStateImpl(nodeHandle, state, result);
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::getStateImpl( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, DataExecution::DataObject& result ) const
    {
        return false;
    }


    /**
     *
     */
    template<typename P>
    Mesh::NodeHandle PclMeshNodesInterface<P>::beginHandle() const
    {
        return Mesh::NodeHandle(0);
    }


    /**
     *
     */
    template<typename P>
    CSIRO::Mesh::NodeHandle PclMeshNodesInterface<P>::endHandle() const
    {
        return Mesh::NodeHandle(pointCloud_->size());
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::getState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, int_type& result ) const
    {
        const point_t& point = getPoint(nodeHandle);
        Q_UNUSED(point)
        return false;
    }


    /**
     *
     */
    template<>
    inline bool PclMeshNodesInterface<pcl::PointXYZRGBNormal>::getState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, int_type& result ) const
    {
        const point_t& point = getPoint(nodeHandle);

        if (&state == rgbaState_)
        {
            uint rgba = 0;
            rgba |= point.r;
            rgba |= point.g << 8;
            rgba |= point.b << 16;
            rgba |= 255 << 24;
            result = static_cast<Mesh::MeshModelInterface::int_type>(rgba);
            return true;
        }

        return false;
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::getState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, double& result ) const
    {
        return false;
    }


    /**
     *
     */
    template<typename P>
    inline bool PclMeshNodesInterface<P>::getState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, Mesh::Vector3d& result ) const
    {
        const point_t& point = getPoint(nodeHandle);
        Q_UNUSED(point)
        return false;
    }


    /**
     *
     */
    template<>
    inline bool PclMeshNodesInterface<pcl::PointXYZRGBNormal>::getState( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, Mesh::Vector3d& result ) const
    {
        const point_t& point = getPoint(nodeHandle);

        if (&state == normalState_)
        {
            result.x = point.normal_x;
            result.y = point.normal_y;
            result.z = point.normal_z;
            return true;
        }

        return false;
    }


    /**
     *
     */
    template<typename P>
    PclMeshNodesInterface<P>* PclMeshNodesInterface<P>::clone() const
    {
        return new PclMeshNodesInterface<P>(*this);
    }


    /**
     *
     */
    template<>
    inline bool PclMeshNodesInterface<pcl::PointXYZRGBNormal>::getStateImpl( const Mesh::NodeHandle& nodeHandle, const Mesh::NodeStateHandle& state, DataExecution::DataObject& result ) const
    {
        if (&result.getFactory() == &DataExecution::DataFactoryTraits<Mesh::MeshModelInterface::int_type>::getInstance())
        {
            getState(nodeHandle, state, result.getRawData<Mesh::MeshModelInterface::int_type>());
        }
        else if (&result.getFactory() == &DataExecution::DataFactoryTraits<Mesh::Vector3d>::getInstance())
        {
            getState(nodeHandle, state, result.getRawData<Mesh::Vector3d>());
        }
        else
        {
            return false;
        }

        return true;
    }


}}

#endif
