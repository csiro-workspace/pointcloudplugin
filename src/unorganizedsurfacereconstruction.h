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

#ifndef CSIRO_POINTCLOUD_UNORGANIZEDSURFACERECONSTRUCTION_H
#define CSIRO_POINTCLOUD_UNORGANIZEDSURFACERECONSTRUCTION_H

#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "pointcloudplugin.h"


namespace CSIRO
{

namespace PointCloud
{
    class UnorganizedSurfaceReconstructionImpl;

  /** \brief An implementation of a greedy triangulation algorithm for 3D points
  * based on local 2D projections. It assumes locally smooth surfaces and relatively smooth transitions between
  * areas with different point densities.
  */
    class CSIRO_POINTCLOUD_API UnorganizedSurfaceReconstruction : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::PCLSurfaceReconstruction)

        UnorganizedSurfaceReconstructionImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
        UnorganizedSurfaceReconstruction(const UnorganizedSurfaceReconstruction&);
        UnorganizedSurfaceReconstruction& operator=(const UnorganizedSurfaceReconstruction&);

    protected:
        virtual bool  execute();

    public:
        UnorganizedSurfaceReconstruction();
        virtual ~UnorganizedSurfaceReconstruction();
    };
}
}

DECLARE_WORKSPACE_OPERATION_FACTORY(CSIRO::PointCloud::UnorganizedSurfaceReconstruction, CSIRO_POINTCLOUD_API)

#endif
