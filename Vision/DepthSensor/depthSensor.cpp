//
//  depthSensor.cpp
//  Robotics and Computer Vision
//
//  Created by Sina Pour Soltani on 13/11/2020.
//  1st semester Master's programme in Advanced Robotics Technology
//

#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
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

using namespace std;

using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::visualization;

using namespace rw::core;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rwlibs::simulation;
using namespace rws;

using rw::graphics::SceneViewer;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;

int main(int argc, char**argv) {
    if(argc < 2){
        cout << "Usage: " << argv[0] << " <scene>" << endl;
        return 0;
    }

    const string WC_FILE = argv[1];
    const WorkCell::Ptr wc = WorkCellLoader::Factory::load(WC_FILE);
    if (wc.isNull())
        RW_THROW("WorkCell could not be loaded.");

    Frame* const camera = wc->findFrame("Camera_Left");
    if (camera == nullptr)
        RW_THROW("Camera frame could not be found.");
    const PropertyMap& properties = camera->getPropertyMap();
    if (!properties.has("Camera"))
        RW_THROW("Camera frame does not have Camera property.");
    const string parameters = properties.get<string>("Camera");
    istringstream iss (parameters, istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    cout << "Camera properties: fov " << fovy << " width " << width << " height " << height << endl;

    cout << "0" << endl;
    RobWorkStudioApp app("app");
    cout << "1" << endl;
    app.start();
    cout << "2" << endl;

    RobWorkStudio* const rwstudio = app.getRobWorkStudio();
    rwstudio->postOpenWorkCell(WC_FILE);
    TimerUtil::sleepMs(5000);

    const SceneViewer::Ptr gldrawer = rwstudio->getView()->getSceneViewer();
    const GLFrameGrabber::Ptr framegrabber = ownedPtr( new GLFrameGrabber(width,height,fovy) );
    framegrabber->init(gldrawer);
    SimulatedCamera::Ptr simcam = ownedPtr(new SimulatedCamera("SimulatedCamera", fovy, camera, framegrabber));
    simcam->setFrameRate(100);
    simcam->initialize();
    simcam->start();
    simcam->acquire();
/*
    static const double DT = 0.001;
    const Simulator::UpdateInfo info(DT);
    State state = wc->getDefaultState();
    int cnt = 0;
    const Image* img;
    while (!simcam->isImageReady()) {
        std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
        simcam->update(info, state);
        cnt++;
    }
    img = simcam->getImage();
    img->saveAsPPM("Image1.ppm");
    simcam->acquire();
    while (!simcam->isImageReady()) {
        std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
        simcam->update(info, state);
        cnt++;
    }
    std::cout << "Took " << cnt << " steps" << std::endl;
    img = simcam->getImage();
    std::cout << "Image: " << img->getWidth() << "x" << img->getHeight() << " bits " << img->getBitsPerPixel() << " channels " << img->getNrOfChannels() << std::endl;
    img->saveAsPPM("Image2.ppm");
*/
    simcam->stop();
    app.close();


    return 0;
}