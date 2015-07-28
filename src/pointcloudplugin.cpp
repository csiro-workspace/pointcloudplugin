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

#include <QString>
#include <QStringList>

#include "Workspace/DataExecution/DataObjects/datafactorytraits.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"
#include "Workspace/Widgets/enumcomboboxfactory.h"
#include "Workspace/Application/LanguageUtils/stringhelpers.h"

#include "Mesh/DataStructures/MeshModelInterface/meshmodelinterfaceregistry.h"

#include "pointcloudplugin.h"
#include "iterativeclosestpoint.h"
#include "smoothesigneddistance.h"
#include "radiusoutlierremoval.h"
#include "mergenearbynodes.h"
#include "organizedsurfacereconstruction.h"
#include "unorganizedsurfacereconstruction.h"
#include "removestatisticaloutliers.h"
#include "movingleastsquares.h"
#include "mlsvoxel.h"
#include "mlssample.h"
#include "mlsrandomuniform.h"
#include "detectholes.h"
#include "normalestimation.h"
#include "poissonreconstruction.h"
#include "pclmeshmodelinterface.h"

#ifndef NO_LIBLAS
#include "lasreader.h"
#endif

namespace CSIRO
{

namespace PointCloud
{
    /**
     * \internal
     */
    class PointCloudPluginImpl
    {
    public:
        // You can add, remove or modify anything in here without
        // breaking binary compatibility. It starts as empty, but
        // leave it here in case at some time in the future you
        // want to add some data for your plugin without breaking
        // binary compatibility.
    };


    /**
     *
     */
    PointCloudPlugin::PointCloudPlugin() :
            CSIRO::Application::WorkspacePlugin("www.csiro.au/workspace/pointcloud",
                                                "Point Cloud",
                                                TOSTRING(POINTCLOUD_PLUGIN_VERSION))
    {
        pImpl_ = new PointCloudPluginImpl;
    }


    /**
     *
     */
    PointCloudPlugin::~PointCloudPlugin()
    {
        delete pImpl_;
    }


    /**
     * \return  The singleton instance of this plugin.
     */
    PointCloudPlugin&  PointCloudPlugin::getInstance()
    {
        // This is a Singleton pattern. There will only ever be one
        // instance of the plugin across the entire application.
        static PointCloudPlugin plugin;
        return plugin;
    }


    /**
     *
     */
    bool  PointCloudPlugin::setup()
    {
        // Add your operation factories like this:
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<MergeNearbyNodes>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<RadiusOutlierRemoval>::getInstance());
        //addFactory( CSIRO::DataExecution::OperationFactoryTraits<OrganizedSurfaceReconstruction>::getInstance() ); needs work
        addFactory( CSIRO::DataExecution::OperationFactoryTraits<UnorganizedSurfaceReconstruction>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<RemoveStatisticalOutliers>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<MovingLeastSquares>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<MlsVoxel>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<MlsSample>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<MlsRandomUniform>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<DetectHoles>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<NormalEstimation>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<PoissonReconstruction>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<SmootheSignedDistance>::getInstance());
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<IterativeClosestPoint>::getInstance());
        
#ifndef NO_LIBLAS
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<LasReader>::getInstance());
#endif
        
        // Add your widget factories like this:
        //addFactory( MyNamespace::MyWidgetFactory::getInstance() );

        Mesh::MeshModelInterfaceRegistry::getInstance().registerType<PclMeshModelInterface>("PCL");

        return true;
    }

    
    /**
     *
     */
    const CSIRO::DataExecution::OperationFactory*  PointCloudPlugin::getAliasedOperationFactory(const QString& opType) const
    {
        // If you rename an operation, you can provide backwards
        // compatibility using something like this (don't forget to
        // include namespaces in the names if relevant):
        //if (opType == "SomeOperationName")
        //    return &CSIRO::DataExecution::OperationFactoryTraits<NewOperationName>::getInstance();

        // If you make use of opType, you can delete the following Q_UNUSED line
        Q_UNUSED(opType);

        // If we get here, opType is not something we renamed, so return a
        // a null pointer to tell the caller
        return static_cast<const CSIRO::DataExecution::OperationFactory*>(0);
    }


    /**
     *
     */
    const CSIRO::DataExecution::DataFactory*  PointCloudPlugin::getAliasedDataFactory(const QString& dataType) const
    {
        // If you rename a data type, you can provide backwards
        // compatibility using something like this (don't forget to
        // include namespaces in the names if relevant):
        //if (dataType == "SomeDataType")
        //    return &CSIRO::DataExecution::DataFactoryTraits<NewDataType>::getInstance();

        // If you make use of dataType, you can delete the following Q_UNUSED line
        Q_UNUSED(dataType);

        // If we get here, dataType is not something we renamed, so return a
        // a null pointer to tell the caller
        return static_cast<const CSIRO::DataExecution::DataFactory*>(0);
    }


    /**
     *	
     */
    QString PointCloudPlugin::getDefaultIconPath() const
    {
        return ":/images/PointCloud/pcl-icon.png";
    }

}}

#ifndef CSIRO_STATIC_BUILD
extern "C"
{
    CSIRO_EXPORTSPEC CSIRO::Application::WorkspacePlugin* getWorkspacePlugin()
    {
        return &CSIRO::PointCloud::PointCloudPlugin::getInstance();
    }
    
    /**
     *	\return The version string for the Workspace build we've been built against
     */
    CSIRO_EXPORTSPEC const char* builtAgainstWorkspace()
    {
        #define STRINGIFY(x) #x
        #define TOSTRING(x) STRINGIFY(x)
        return TOSTRING(CSIRO_WORKSPACE_VERSION_CHECK);
    }
}
#endif
