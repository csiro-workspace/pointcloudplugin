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

#include "Mesh/Geometry/vector3d.h"
#include "pclmeshnodesinterface.h"


namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{
    namespace
    {
        static const DataFactory* vector3dDataFactory   = &DataFactoryTraits<Vector3d>::getInstance();
    }


    /**
     * 
     */
    void PclMeshNodesInterface::initStates()
    {
        registerExistingState(0, "RGBA", TypedObject<MeshModelInterface::int_type>(0xFFFFFFFF));
        registerExistingState(1, "normal", TypedObject<Vector3d>(Vector3d(0,0,0)));

        rgbaState_ = &getStateHandle("RGBA");
        normalState_ = &getStateHandle("normal");
    }


    /**
     * 
     */
    PclMeshNodesInterface::PclMeshNodesInterface() :
        pointCloud_(new pointcloud_t()),
        rgbaState_(0),
        normalState_(0)
    {
        initStates();
    }


    /**
     * 
     */
    PclMeshNodesInterface::PclMeshNodesInterface( const PclMeshNodesInterface& other ) :
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
    const PclMeshNodesInterface& PclMeshNodesInterface::operator=( const PclMeshNodesInterface& other )
    {
        MeshNodesInterface::operator=(other);
        pointCloud_ = pointcloud_t::Ptr(new pointcloud_t(*other.pointCloud_));
        return *this;
    }


    /**
     * 
     */
    NodeHandle PclMeshNodesInterface::add( const Vector3d& position )
    {
        size_type newIndex = pointCloud_->size();
        point_t point;
        point.x = position.x;
        point.y = position.y;
        point.z = position.z;
        pointCloud_->push_back(point);
        return NodeHandle(newIndex);
    }


    /**
     * 
     */
    Vector3d PclMeshNodesInterface::getPosition( const NodeHandle& nodeHandle ) const
    {
        assert(nodeHandle.getIndex() < static_cast<size_type>(positions_.size()));
        const point_t& point = getPoint(nodeHandle);
        Vector3d result(point.x, point.y, point.z);
        return result;
    }


    /**
     * 
     */
    void PclMeshNodesInterface::setPosition( const NodeHandle& nodeHandle, const Vector3d& position )
    {
        assert(nodeHandle.getIndex() < static_cast<size_type>(positions_.size()));
        point_t& point = getPoint(nodeHandle);
        point.x = position.x;
        point.y = position.y;
        point.z = position.z;
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::isStateTypeSupported( const DataExecution::DataFactory& dataFactory ) const
    {
        return true;
    }


    /**
     * 
     */
    MeshNodesInterface::size_type PclMeshNodesInterface::size() const
    {
        return pointCloud_->size();
    }


    /**
     * 
     */
    void PclMeshNodesInterface::clear()
    {
        pointCloud_->clear();
    }


    /**
     * 
     */
    PclMeshNodesInterface::iterator PclMeshNodesInterface::erase( const iterator& position )
    {
        pointCloud_->erase(pointCloud_->begin() + position->getIndex());
        return createIterator(NodeHandle(position->getIndex() + 1));
    }


    /**
     * 
     */
    void PclMeshNodesInterface::remove( const NodeHandle& nodeHandle )
    {
        pointCloud_->erase(pointCloud_->begin() + nodeHandle.getIndex());
    }

    
    /**
     * 
     */
    bool PclMeshNodesInterface::empty() const
    {
        return pointCloud_->empty();
    }


    /**
     * 
     */
    void PclMeshNodesInterface::reserve( size_type n )
    {
        pointCloud_->reserve(n);
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::setState( 
        const NodeHandle& nodeHandle, 
        const NodeStateHandle& state, 
        const DataExecution::DataObject& value )
    {
        if (&value.getFactory() == &DataFactoryTraits<MeshModelInterface::int_type>::getInstance())
        {
            setState(nodeHandle, state, value.getRawData<MeshModelInterface::int_type>());
        }
        else if (&value.getFactory() == &DataFactoryTraits<Vector3d>::getInstance())
        {
            setState(nodeHandle, state, value.getRawData<Vector3d>());
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
    bool PclMeshNodesInterface::setState( const NodeHandle& nodeHandle, const NodeStateHandle& state, int_type value )
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
    bool PclMeshNodesInterface::setState( const NodeHandle& nodeHandle, const NodeStateHandle& state, double value )
    {
        return false;
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::setState( const NodeHandle& nodeHandle, const NodeStateHandle& state, const Vector3d& value )
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
    NodeHandle PclMeshNodesInterface::advance( const NodeHandle& nodeHandle, size_type n ) const
    {
        return NodeHandle(nodeHandle.getIndex() + n);
    }


    /**
     * 
     */
    NodeStateHandle* PclMeshNodesInterface::addStateToImplementation( const QString& name, 
                                                                      const DataExecution::DataObject& defaultValue )
    {
        return 0; // not supported
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::removeStateFromImplementation( const NodeStateHandle& state )
    {
        return false; // not supported
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::getStateImpl( const NodeHandle& nodeHandle, const NodeStateHandle& state, DataExecution::DataObject& result )
    {
        // Use const version - base class that calls this will ensure data isn't shared which is the only difference b/w const and non-const version
        return static_cast<const PclMeshNodesInterface*>(this)->getStateImpl(nodeHandle, state, result);
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::getStateImpl( const NodeHandle& nodeHandle, const NodeStateHandle& state, DataExecution::DataObject& result ) const
    {
        if (&result.getFactory() == &DataFactoryTraits<MeshModelInterface::int_type>::getInstance())
        {
            getState(nodeHandle, state, result.getRawData<MeshModelInterface::int_type>());
        }
        else if (&result.getFactory() == &DataFactoryTraits<Vector3d>::getInstance())
        {
            getState(nodeHandle, state, result.getRawData<Vector3d>());
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
    NodeHandle PclMeshNodesInterface::beginHandle() const
    {
        return NodeHandle(0);
    }


    /**
     * 
     */
    CSIRO::Mesh::NodeHandle PclMeshNodesInterface::endHandle() const
    {
        return NodeHandle(pointCloud_->size());
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::getState( const NodeHandle& nodeHandle, const NodeStateHandle& state, int_type& result ) const
    {
        const point_t& point = getPoint(nodeHandle);

        if (&state == rgbaState_)
        {
            uint rgba = 0;

            rgba |= point.r;
            rgba |= point.g << 8;
            rgba |= point.b << 16;
            rgba |= 255 << 24;
            
            result = static_cast<MeshModelInterface::int_type>(rgba);
            return true;
        }

        return false;
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::getState( const NodeHandle& nodeHandle, const NodeStateHandle& state, double& result ) const
    {
        return false;
    }


    /**
     * 
     */
    bool PclMeshNodesInterface::getState( const NodeHandle& nodeHandle, const NodeStateHandle& state, Vector3d& result ) const
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
}}

