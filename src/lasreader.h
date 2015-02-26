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

#ifndef CSIRO_POINTCLOUD_LASREADER_H
#define CSIRO_POINTCLOUD_LASREADER_H

#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "pointcloudplugin_api.h"


namespace CSIRO
{

namespace PointCloud
{
    class LasReaderImpl;

    /**
     * \brief Read a PCD file
     */
    class CSIRO_POINTCLOUD_API LasReader : public DataExecution::Operation
    {
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::LasReader)

        LasReaderImpl*  pImpl_;

        LasReader(const LasReader&);
        LasReader& operator=(const LasReader&);

    protected:
        virtual bool  execute();

    public:
        LasReader();
        virtual ~LasReader();
    };

}}   // end of namespaces

DECLARE_WORKSPACE_OPERATION_FACTORY(CSIRO::PointCloud::LasReader, CSIRO_POINTCLOUD_API)

#endif
