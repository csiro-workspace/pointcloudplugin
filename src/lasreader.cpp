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
#include <liblas/liblas.hpp>

#include <QFileInfo>

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"
#include "Workspace/Application/System/systemutilities.h"

#include "pointcloudplugin.h"
#include "pclmeshmodelinterface.h"
#include "pclmeshnodesinterface.h"

#include "lasreader.h"


namespace CSIRO
{
    using namespace Mesh;
    using namespace DataExecution;

namespace PointCloud
{

    /**
     * \internal
     */
    class LasReaderImpl
    {
        Q_DECLARE_TR_FUNCTIONS(CSIRO::PointCloud::LasReaderImpl)

    public:
        LasReader& op_;

        TypedObject<QString>            fileName_;
        TypedObject<MeshModelInterface> mesh_;

        InputScalar                     inputFileName_;
        Output                          outputMesh_;

        LasReaderImpl(LasReader& op);
    };


    /**
     *
     */
    LasReaderImpl::LasReaderImpl(LasReader& op) :
        op_(op),
        inputFileName_("File name", fileName_, op_),
        outputMesh_("Mesh model", mesh_, op)
    {
        inputFileName_.setPreferredWidget("CSIRO::Widgets::FileNameWidget");
    }


    /**
     *
     */
    LasReader::LasReader() :
        Operation(OperationFactoryTraits<LasReader>::getInstance(), tr("Read LAS/LAZ file")),
        pImpl_(static_cast<LasReaderImpl*>(0))
    {
        pImpl_ = new LasReaderImpl(*this);
    }


    /**
     *
     */
    LasReader::~LasReader()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  LasReader::execute()
    {
        PclMeshModelInterface* meshInterface = new PclMeshModelInterface();
        pImpl_->mesh_.setData(meshInterface, true);

        QString filename = CSIRO::System::Utilities::downloadIfRemote(*pImpl_->fileName_, getLabel());

        if (!QFileInfo(filename).exists())
        {
            logText(tr("ERROR: %1 does not exist\n").arg(*pImpl_->fileName_));
            return false;
        }
        
        std::ifstream ifs;
        ifs.open(filename.toLatin1().constData(), std::ios::in | std::ios::binary);

        try
        {
            liblas::ReaderFactory f;
            liblas::Reader reader = f.CreateWithStream(ifs);
            liblas::Header const& header = reader.GetHeader();

            logText(tr("Reading: %1\n").arg(*pImpl_->fileName_));
            logText(tr("Compressed: %1\n").arg((header.Compressed() == true) ? "true":"false"));
            logText(tr("Signature: %1\n").arg(header.GetFileSignature().c_str()));
            logText(tr("Points count: %1\n").arg(header.GetPointRecordsCount()));

            PclMeshNodesInterface::pointcloud_t::Ptr points = meshInterface->getPointCloud();
            points->reserve(header.GetPointRecordsCount());

            int foo = 0;
            while (reader.ReadNextPoint())
            {
                liblas::Point const& p = reader.GetPoint();
                PclMeshNodesInterface::point_t pOut = PclMeshNodesInterface::point_t();
                pOut.x = p.GetX();
                pOut.y = p.GetY();
                pOut.z = p.GetZ();
                liblas::Color color = p.GetColor();
                pOut.r = static_cast<int>(int(255) * color.GetRed() / std::numeric_limits<liblas::Color::value_type>::max());
                pOut.g = static_cast<int>(int(255) * color.GetGreen() / std::numeric_limits<liblas::Color::value_type>::max());
                pOut.b = static_cast<int>(int(255) * color.GetBlue() / std::numeric_limits<liblas::Color::value_type>::max());
                points->push_back(pOut);
            }
        }
        catch (const std::runtime_error& e)
        {
            logText(tr("ERROR: std::runtime_error - %1\n").arg(e.what()));
            return false;
        }

        return true;
    }

}}   // end of namespaces


using namespace CSIRO::PointCloud;
using namespace CSIRO::DataExecution;
DEFINE_WORKSPACE_OPERATION_FACTORY(LasReader, PointCloudPlugin::getInstance(), Operation::tr("PointCloud"))
