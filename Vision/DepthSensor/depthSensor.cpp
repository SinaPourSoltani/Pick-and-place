//
//  depthSensor.cpp
//  Robotics and Computer Vision
//
//  Created by Sina Pour Soltani on 13/11/2020.
//  1st semester Master's programme in Advanced Robotics Technology
//

#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/core/PropertyMap.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <thread>

#define Z_MIN -2

using namespace std;

using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::visualization;

using namespace rw::core;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rws;

using rw::graphics::SceneViewer;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;

PointCloud<PointXYZ>::Ptr grapPointCloud(WorkCell::Ptr wc, RobWorkStudio* rwstudio, Frame* cameraFrame, double fovy, int width, int height){
    // Inspiration from: https://www.robwork.dk/manual/simulated_sensors/
    PointCloud<PointXYZ>::Ptr pclScene(new PointCloud<PointXYZ>);
    
    State state = wc->getDefaultState();
    SceneViewer::Ptr gldrawer = rwstudio->getView()->getSceneViewer();
    GLFrameGrabber25D::Ptr framegrabber25D = ownedPtr( new GLFrameGrabber25D(width,height,fovy) );
    framegrabber25D->init(gldrawer);
    
    framegrabber25D->grab(cameraFrame, state);
    rw::geometry::PointCloud* img = &(framegrabber25D->getImage());
    for(auto &p : img->getData()) {
        if (p(2) > Z_MIN) {
            PointXYZ pcl_p;
            pcl_p.x = p(0);
            pcl_p.y = p(1);
            pcl_p.z = p(2);
            pclScene->push_back(pcl_p);
        }
    }
    return pclScene;
}


PointCloud<PointNormal>::Ptr calculateSurfaceNormals(PointCloud<PointXYZ>::Ptr cloud){
    // Inspiration from: https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    
    // Compute the features
    ne.compute (*cloud_normals);
    
    // cloud_normals->size () should have the same size as the input cloud->size ()*
    
    PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
    concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
    
    return cloud_with_normals;
}




int main(int argc, char**argv) {
    if(argc < 2){
        cout << "Usage: " << argv[0] << " <scene>" << endl;
        return 0;
    }

    string WC_FILE = argv[1];
    string camera25D = "Scanner25D";
    
    // Load workcell
    const WorkCell::Ptr wc = WorkCellLoader::Factory::load(WC_FILE);
    if (wc.isNull())
        RW_THROW("WorkCell could not be loaded.");
    
    // Load 25D camera frame
    Frame* const cameraFrame = wc->findFrame(camera25D);
    if (cameraFrame == nullptr)
        RW_THROW("Camera frame could not be found.");
    
    // Read camera properties from <scene>
    const PropertyMap& properties = cameraFrame->getPropertyMap();
    if (!properties.has(camera25D))
        RW_THROW("Camera frame does not have Camera property.");
    const string parameters = properties.get<string>(camera25D);
    
    istringstream iss (parameters, istringstream::in);
    double fovy; int width, height;
    iss >> fovy >> width >> height;
    cout << "Camera properties: fov " << fovy << " width " << width << " height " << height << endl;
    
    
    // Initialized globally to allow for main to visualize when thread overwrites with simulated point cloud
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointCloud<PointNormal>::Ptr pclToVisualize(new PointCloud<PointNormal>);
    
    RobWorkStudioApp app("");
    thread t([&]() {
        // Wait for app to launch
        while (!app.isRunning()) {
            TimerUtil::sleepMs(1000);
            cout << "Waiting for launch.." << endl;
        }
        RobWorkStudio* rwstudio = app.getRobWorkStudio();
        rwstudio->postOpenWorkCell(WC_FILE);
        TimerUtil::sleepMs(2000);
        
        PointCloud<PointXYZ>::Ptr pclScene = grapPointCloud(wc, rwstudio, cameraFrame, fovy, width, height);
        cloud = pclScene;
        PointCloud<PointNormal>::Ptr pclNormals = calculateSurfaceNormals(pclScene);
        pclToVisualize = pclNormals;
        app.close();
    });
    t.detach();
    app.run();
    

    
    PCLVisualizer v("PointCloud");
    v.addPointCloud<PointXYZ>(cloud, "points");
    v.addPointCloudNormals<PointNormal>(pclToVisualize,100,0.06,"normals");
    v.spin();

    return 0;
}
