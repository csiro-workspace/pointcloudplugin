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

#include "Mesh/DataStructures/MeshModelInterface/meshnodesinterface.h"

namespace CSIRO
{
 
namespace PointCloud
{
    /**
     * \brief A Workspace MeshNodesInterface wrapper around a PCL cloud
     */
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

        typedef pcl::PointXYZRGBNormal      point_t;
        typedef pcl::PointCloud<point_t>    pointcloud_t;
        pointcloud_t::Ptr getPointCloud() { return pointCloud_; }

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

        pointcloud_t::Ptr            pointCloud_;
        const Mesh::NodeStateHandle* rgbaState_;
        const Mesh::NodeStateHandle* normalState_;
    };
}}

#endif
