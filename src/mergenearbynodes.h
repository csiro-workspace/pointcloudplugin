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

#ifndef CSIRO_POINTCLOUD_MERGENEARBYNODES_H
#define CSIRO_POINTCLOUD_MERGENEARBYNODES_H

#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "Mesh/DataStructures/octree.h"

#include "pointcloudplugin.h"


namespace CSIRO
{

namespace PointCloud
{
    class MergeNearbyNodesImpl;

    /**
     * \brief Merge nearby nodes within a given radius
     */
    class CSIRO_POINTCLOUD_API MergeNearbyNodes : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::MergeNearbyNodes)

        MergeNearbyNodesImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
        MergeNearbyNodes(const MergeNearbyNodes&);
        MergeNearbyNodes& operator=(const MergeNearbyNodes&);


        /**
         *	\internal
         */
        struct Particle
        {
            typedef double ScalarType;

            uint                        id_;
            Mesh::OctreeVector3<double> position_;

            Particle() : id_(0) {}
            Particle(uint id, const Mesh::Vector3d& pos) : id_(id), position_(pos)  {}
            const Mesh::OctreeVector3<double>& getPosition() const { return position_; }
            double getRadius() const { return 0; }
        };

        friend class MergeNearbyNodesImpl;
        friend class RadiusOutlierRemovalImpl;//Uses the octree particles

    protected:
        virtual bool  execute();

    public:
        MergeNearbyNodes();
        virtual ~MergeNearbyNodes();
    };
}
}

DECLARE_WORKSPACE_OPERATION_FACTORY(CSIRO::PointCloud::MergeNearbyNodes, CSIRO_POINTCLOUD_API)

#endif

