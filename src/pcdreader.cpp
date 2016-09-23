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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QFileInfo>

#include "Workspace/Application/System/systemutilities.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"

#include "pointcloudplugin.h"
#include "pclmeshmodelinterface.h"
#include "pclmeshnodesinterface.h"

#include "pcdreader.h"


namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{

    /**
     * \internal
     */
    class PcdReaderImpl
    {
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::PcdReaderImpl)

    public:
        PcdReader& op_;

        TypedObject<QString>            fileName_;
        TypedObject<MeshModelInterface> mesh_;

        InputScalar                     inputFileName_;
        Output                          outputMesh_;

        PcdReaderImpl(PcdReader& op);
    };


    /**
     *
     */
    PcdReaderImpl::PcdReaderImpl(PcdReader& op) :
        op_(op),
        inputFileName_("File name", fileName_, op_),
        outputMesh_("Mesh model", mesh_, op)
    {
        inputFileName_.setPreferredWidget("CSIRO::Widgets::FileNameWidget");
    }


    /**
     *
     */
    PcdReader::PcdReader() :
        Operation(OperationFactoryTraits<PcdReader>::getInstance(), tr("Read PCD point cloud file")),
        pImpl_(static_cast<PcdReaderImpl*>(0))
    {
        pImpl_ = new PcdReaderImpl(*this);
    }


    /**
     *
     */
    PcdReader::~PcdReader()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  PcdReader::execute()
    {
        QString fileName = CSIRO::System::Utilities::downloadIfRemote(*pImpl_->fileName_, getLabel());

        PclMeshModelInterface* meshInterface = new PclMeshModelInterface();
        pImpl_->mesh_.setData(meshInterface, true);
        
        if (pcl::io::loadPCDFile<PclMeshNodesInterface::point_t>(fileName.toStdString(), *meshInterface->getPointCloud()) != 0)
        {
            logText(tr("ERROR: loadPCDFile() failed for %1 %2\n").arg(fileName).arg(*pImpl_->fileName_));
            return false;
        }

        return true;
    }

}}   // end of namespaces


using namespace CSIRO::PointCloud;
using namespace CSIRO::DataExecution;
DEFINE_WORKSPACE_OPERATION_FACTORY(PcdReader, PointCloudPlugin::getInstance(), Operation::tr("PointCloud"))
