//
//  sparseStereo.cpp
//  Robotics and Computer Vision
//
//  Created by Jens O. H. Iversen on 11/11/2020.
//  Inspired by the RobWorkStudio documentation: https://www.robwork.dk/manual/simulated_sensors/
//  1st semester Master's programme in Advanced Robotics Technology
//

#include <iostream>
#include <string>

#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/core/PropertyMap.hpp>

using namespace std;

using namespace rw::core;
using namespace rw::common;
using rw::graphics::SceneViewer;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;
using namespace rwlibs::simulation;
using namespace rws;


int main(int argc, char** argv) {

  //How to call the script
  if(argc < 3){
    cout << "Usage: " << argv[0] << " <scene> <camera>" << endl;
    return 0;
  }
  //Loading workcell
  const WorkCell::Ptr wc = WorkCellLoader::Factory::load(argv[1]);
  if(wc.isNull()){
    RW_THROW("WorkCell could not be loaded.");
  }
  //Loading camera frame
  Frame* const camera = wc->findFrame(argv[2]);
  if(camera == nullptr){
    RW_THROW("Camera frame could not be found.");
  }
  //Loading properties of camera
  const PropertyMap& properties = camera->getPropertyMap();
  if (!properties.has("Camera")){
    RW_THROW("Camera frame does not have Camera property.");
  }
  //Printing the properties of the camera
  const string parameters = properties.get<string>("Camera");
  istringstream iss (parameters, istringstream::in);
  double fovy, width, height;
  iss >> fovy >> width >> height;
  cout << "Camera properties: fov " << fovy << " width " << width << " height " << height << endl;


  RobWorkStudioApp app("");
  app.start();

  RobWorkStudio* const rwstudio = app.getRobWorkStudio();
  rwstudio->postOpenWorkCell(argv[1]);
  TimerUtil::sleepMs(5000);


  const SceneViewer::Ptr gldrawer = rwstudio->getView()->getSceneViewer();
  const GLFrameGrabber::Ptr framegrabber = ownedPtr(new GLFrameGrabber(width,height,fovy));
  framegrabber->init(gldrawer);
  SimulatedCamera::Ptr simcam = ownedPtr(new SimulatedCamera("SimulatedCamera", fovy, camera, framegrabber));
  simcam->setFrameRate(100);
  simcam->initialize();
  simcam->start();
  simcam->acquire();


  static const double DT = 0.001;
  const Simulator::UpdateInfo info(DT);
  State state = wc->getDefaultState();
  int cnt = 0;
  const Image* img;
  while(!simcam->isImageReady()){
    cout << "Image is not ready yet. Iteration " << cnt << endl;
    simcam->update(info,state);
    cnt++;
  }
  img = simcam->getImage();
  img->saveAsPPM("Image.ppm");
  cout << "Took " << cnt << " steps" << endl;
  cout << "Image: " << img->getWidth() << "x" << img->getHeight() << endl;

  simcam->stop();
  app.close();





















  return 0;
}
