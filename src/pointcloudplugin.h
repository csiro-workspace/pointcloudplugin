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

#ifndef CSIRO_POINTCLOUD_POINTCLOUDPLUGIN_H
#define CSIRO_POINTCLOUD_POINTCLOUDPLUGIN_H

#include "Workspace/Application/workspaceplugin.h"

#include "pointcloudplugin_api.h"


namespace CSIRO
{

namespace PointCloud
{
    class PointCloudPluginImpl;

    /**
     * \brief CSIRO Workspace plugin for processing point cloud data
     */
    class CSIRO_POINTCLOUD_API PointCloudPlugin : public CSIRO::Application::WorkspacePlugin
    {
        PointCloudPluginImpl*  pImpl_;

        PointCloudPlugin();
        ~PointCloudPlugin();

        // Prevent copying and assignment
        PointCloudPlugin(const PointCloudPlugin&);
        PointCloudPlugin& operator=(const PointCloudPlugin&);

    protected:
        virtual const CSIRO::DataExecution::DataFactory*       getAliasedDataFactory(const QString& dataType) const;
        virtual const CSIRO::DataExecution::OperationFactory*  getAliasedOperationFactory(const QString& opType) const;

    public:
        static PointCloudPlugin&  getInstance();

        virtual QString  getDefaultIconPath() const;

        virtual bool  setup();
    };
}}

#endif
