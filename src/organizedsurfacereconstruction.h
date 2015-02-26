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

#ifndef CSIRO_POINTCLOUD_ORGANIZEDSURFACERECONSTRUCTION_H
#define CSIRO_POINTCLOUD_ORGANIZEDSURFACERECONSTRUCTION_H

#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "pointcloudplugin.h"


namespace CSIRO
{

namespace PointCloud
{
    class OrganizedSurfaceReconstructionImpl;

    /**
     * \brief Put a one-line description of your operation here
     *
     * Add a more detailed description of your operation here
     * or remove these lines if the brief description above
     * is sufficient.
     */
    class CSIRO_POINTCLOUD_API OrganizedSurfaceReconstruction : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::PCLSufaceReconstruction)

        OrganizedSurfaceReconstructionImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
        OrganizedSurfaceReconstruction(const OrganizedSurfaceReconstruction&);
        OrganizedSurfaceReconstruction& operator=(const OrganizedSurfaceReconstruction&);

    protected:
        virtual bool  execute();

    public:
        OrganizedSurfaceReconstruction();
        virtual ~OrganizedSurfaceReconstruction();
    };
}
}

DECLARE_WORKSPACE_OPERATION_FACTORY(CSIRO::PointCloud::OrganizedSurfaceReconstruction, CSIRO_POINTCLOUD_API)

#endif
