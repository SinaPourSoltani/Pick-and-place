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

#define LEFT "left"
#define RIGHT "right"

class SparseStereo{
  private:
    WorkCell::Ptr wc;
    Frame* leftCamera;
    Frame* rightCamera;
    PropertyMap leftCameraProperties;
    PropertyMap rightCameraProperties;

    string scene, leftCam, rightCam;

    double leftFovy, rightFovy;
    int leftWidth, leftHeight, rightWidth, rightHeight;

  public:
    SparseStereo(char** argv){
      wc = WorkCellLoader::Factory::load(argv[1]);
      if(wc.isNull()){RW_THROW("WorkCell could not be loaded.");}
      scene = argv[1];

      leftCamera = wc->findFrame(argv[2]);
      if(leftCamera == nullptr){RW_THROW("Left camera frame could not be found.");}
      leftCam = argv[2];

      rightCamera = wc->findFrame(argv[3]);
      if(rightCamera == nullptr){RW_THROW("Right camera frame could not be found.");}
      rightCam = argv[3];

      leftCameraProperties = leftCamera->getPropertyMap();
      if (!leftCameraProperties.has("Camera")){RW_THROW("Left camera frame does not have Camera property.");}

      rightCameraProperties = rightCamera->getPropertyMap();
      if (!rightCameraProperties.has("Camera")){RW_THROW("Right camera frame does not have Camera property.");}

      //Printing the properties of the camera
      const string leftCameraParameters = leftCameraProperties.get<string>("Camera");
      istringstream iss (leftCameraParameters, istringstream::in);
      iss >> leftFovy >> leftWidth >> leftHeight;
      cout << "Left camera properties:  fov " << leftFovy << " width " << leftWidth << " height " << leftHeight << endl;
      const string rightCameraParameters = rightCameraProperties.get<string>("Camera");
      iss = istringstream(rightCameraParameters, istringstream::in);
      iss >> rightFovy >> rightWidth >> rightHeight;
      cout << "Right camera properties: fov " << rightFovy << " width " << rightWidth << " height " << rightHeight << endl;
    }

    void takeImages(){
      RobWorkStudioApp app("");
      app.start();

      RobWorkStudio* const rwstudio = app.getRobWorkStudio();
      rwstudio->postOpenWorkCell(scene);
      TimerUtil::sleepMs(5000);

      //Setting up for leftCamera
      const SceneViewer::Ptr leftGldrawer = rwstudio->getView()->getSceneViewer();
      const GLFrameGrabber::Ptr leftFramegrabber = ownedPtr(new GLFrameGrabber(leftWidth,leftHeight,leftFovy));
      leftFramegrabber->init(leftGldrawer);
      SimulatedCamera::Ptr leftSimcam = ownedPtr(new SimulatedCamera("SimulatedCamera", leftFovy, leftCamera, leftFramegrabber));
      leftSimcam->setFrameRate(100);
      leftSimcam->initialize();
      leftSimcam->start();
      leftSimcam->acquire();


      static const double DT = 0.001;
      const Simulator::UpdateInfo leftInfo(DT);
      State state = wc->getDefaultState();
      int cnt = 0;
      const Image* img;
      while(!leftSimcam->isImageReady()){
        cout << "Image is not ready yet. Iteration " << cnt << endl;
        leftSimcam->update(leftInfo,state);
        cnt++;
      }
      img = leftSimcam->getImage();
      img->saveAsPPM("Images/left_image.ppm");
      cout << "Took " << cnt << " steps" << endl;
      cout << "Left image: " << img->getWidth() << "x" << img->getHeight() << endl;
      leftSimcam->stop();

      //Setting up for rightCamera
      const SceneViewer::Ptr rightGldrawer = rwstudio->getView()->getSceneViewer();
      const GLFrameGrabber::Ptr rightFramegrabber = ownedPtr(new GLFrameGrabber(rightWidth,rightHeight,rightFovy));
      rightFramegrabber->init(rightGldrawer);
      SimulatedCamera::Ptr rightSimcam = ownedPtr(new SimulatedCamera("SimulatedCamera", rightFovy, rightCamera, rightFramegrabber));
      rightSimcam->setFrameRate(100);
      rightSimcam->initialize();
      rightSimcam->start();
      rightSimcam->acquire();

      const Simulator::UpdateInfo rightInfo(DT);
      cnt = 0;
      while(!rightSimcam->isImageReady()){
        cout << "Image is not ready yet. Iteration " << cnt << endl;
        rightSimcam->update(rightInfo,state);
        cnt++;
      }
      img = rightSimcam->getImage();
      img->saveAsPPM("Images/right_image.ppm");
      cout << "Took " << cnt << " steps" << endl;
      cout << "Right image: " << img->getWidth() << "x" << img->getHeight() << endl;
      rightSimcam->stop();

      app.close();
    }

    //virtual ~SparseStereo();
};



int main(int argc, char** argv) {

  //How to call the script
  if(argc < 4){
    cout << "Usage: " << argv[0] << " <scene> <left camera> <right camera>" << endl;
    return 0;
  }

  SparseStereo sparseStereo(argv);
  sparseStereo.takeImages();



  return 0;
}
