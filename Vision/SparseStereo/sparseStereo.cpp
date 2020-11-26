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
#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>


#include "methods.hpp"


using namespace std;
using namespace cv;

using namespace rw::math;
using namespace rw::core;
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using rw::graphics::SceneViewer;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace rwlibs::proximitystrategies;
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
    string rightImageStr = "right_image.png";
    string leftImageStr = "left_image.png";
    string pathRightImage = "Images/"+rightImageStr;
    string pathLeftImage = "Images/"+leftImageStr;
    string leftImageNoiseStr = "left_image_noise.png";
    string rightImageNoiseStr = "right_image_noise.png";
    string pathLeftImageNoise = "Images/"+leftImageNoiseStr;
    string pathRightImageNoise = "Images/"+rightImageNoiseStr;

    double leftFovy, rightFovy;
    int leftWidth, leftHeight, rightWidth, rightHeight;

    State state;

    Eigen::Matrix<double, 3, 4> leftProjectionMatrix;
    Eigen::Matrix<double, 3, 4> rightProjectionMatrix;



    //This funciton is inspired from Sina's work
    void addNoiseToImage(string image_, string output, int mean = 0, double variance = 0.1){
      cout << "\nAdding noise to " << image_ << endl;
      Mat img = imread(image_, 1);
      Scalar a = cv::mean(img); //Must have cv:: before or it wont compile. I think i takes another mean function from RobWorkStudio or something.
      float power = (a.val[0] + a.val[1] + a.val[2]) / 3;
      Mat noise(img.size(), img.type());
      randn(noise, mean, power * variance);
      addWeighted(img, 1.0, noise, 1.0, 0.0, img);
      imwrite(output, img);
    }

    //This function is inspired from the SamplePlugin given for the project
    Eigen::Matrix<double, 3, 4> calculateProjectionMatrix(Frame* cam_, double fovy_, double width_, double height_){

      double fovyPixel = height_ / 2 / tan(fovy_ * (2*M_PI) / 360.0 / 2.0);
      //cout << "Fovy: " << fovy_ << endl;
      //cout << "Fovy pixel: " << fovyPixel << endl;

      Eigen::Matrix<double, 3, 4> KA;
      KA << fovyPixel, 0, width_ / 2.0, 0,
            0, fovyPixel, height_ / 2.0, 0,
            0, 0, 1, 0;

      //cout << "Intrinsic parameters (KA): " << endl;
      //cout << KA << endl;

      Transform3D<> camPosOriToWorld = cam_->wTf(state);
      Transform3D<> H = inverse(camPosOriToWorld * inverse(Transform3D<>(RPY<>(-M_PI, 0, M_PI).toRotation3D())));

      //cout << "Extrinsic parameters (H): " << endl;
      //cout << H.e() << endl;

      //H skal muligvis inverse.
      Eigen::Matrix<double, 3, 4> projectionMatrix = KA * H.e();
      //cout << "Projection matrix: " << endl;
      //cout << projectionMatrix << endl;
      return projectionMatrix;
    }

    //This function is inspired from the SamplePlugin given for the project
    void saveAsOpenCVImage(Mat image, string path){
      Mat imflip, imflip_mat;
      flip(image, imflip, 1);
      cvtColor(imflip, imflip_mat, COLOR_RGB2BGR);
      imwrite(path, imflip_mat);
    }

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
      //cout << "Left camera properties:  fov " << leftFovy << " width " << leftWidth << " height " << leftHeight << endl;
      const string rightCameraParameters = rightCameraProperties.get<string>("Camera");
      iss = istringstream(rightCameraParameters, istringstream::in);
      iss >> rightFovy >> rightWidth >> rightHeight;
      //cout << "Right camera properties: fov " << rightFovy << " width " << rightWidth << " height " << rightHeight << endl;

      state = wc->getDefaultState();
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
      state = wc->getDefaultState();
      int cnt = 0;
      const Image* img;
      while(!leftSimcam->isImageReady()){
        cout << "Image is not ready yet. Iteration " << cnt << endl;
        leftSimcam->update(leftInfo,state);
        cnt++;
      }
      img = leftSimcam->getImage();
      //img->saveAsPPM(pathLeftImage);
      //These two lines are taken from the SamplePlugin
      Mat leftImage = Mat(img->getHeight(), img->getWidth(), CV_8UC3, (Image*)img->getImageData());
      saveAsOpenCVImage(leftImage, pathLeftImage);


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
      //img->saveAsPPM(pathRightImage);
      //These two lines are taken from the SamplePlugin
      Mat rightImage = Mat(img->getHeight(), img->getWidth(), CV_8UC3, (Image*)img->getImageData());
      saveAsOpenCVImage(rightImage, pathRightImage);
      cout << "Took " << cnt << " steps" << endl;
      cout << "Right image: " << img->getWidth() << "x" << img->getHeight() << endl;
      rightSimcam->stop();

      app.close();
    }

    void addNoiseToImages(int mean = 0, double variance = 0.1){
      addNoiseToImage(pathLeftImage, pathLeftImageNoise, mean, variance);
      addNoiseToImage(pathRightImage, pathRightImageNoise, mean, variance);
    }

    //This function is temp until feature detection is applied.
    array<Mat,2> openImages(){
      array<Mat,2> points;
      Mat leftImg = imread(pathLeftImageNoise, CV_LOAD_IMAGE_COLOR);
      points[0] = getMouseClick("Left image.", leftImg);
      //cout << "Left point: \n" << points[0] << endl;

      Mat rightImg = imread(pathRightImageNoise, CV_LOAD_IMAGE_COLOR);
      points[1] = getMouseClick("Right image.", rightImg);
      //cout << "Right point: \n" << points[1] << endl;

      return points;
    }

    void stereopsis(){
      leftProjectionMatrix = calculateProjectionMatrix(leftCamera, leftFovy, leftWidth, leftHeight);
      rightProjectionMatrix = calculateProjectionMatrix(rightCamera, rightFovy, rightWidth, rightHeight);

      Mat leftProjectionMat = convertMatrixToMat(leftProjectionMatrix);
      Mat rightProjectionMat = convertMatrixToMat(rightProjectionMatrix);

      auto leftPp = splitProjectionMatrix(leftProjectionMat);
      auto rightPp = splitProjectionMatrix(rightProjectionMat);

      auto leftOC = computeOpticalCenter(leftPp);
      auto rightOC = computeOpticalCenter(rightPp);

      auto leftEpipole = computeEpipole(leftProjectionMat, rightOC);
      auto rightEpipole = computeEpipole(rightProjectionMat, leftOC);

      //Fundamental matrix left to right
      auto fundMatLeftToRight = computeFundamentalMatrix(rightEpipole, rightProjectionMat, leftProjectionMat);

      //This will be replaced with feature detection
      auto points = openImages();

      auto leftMInf = projectToInf(leftPp[0], points[0]);
      auto rightMInf = projectToInf(rightPp[0], points[1]);

      auto leftPluckerLine = computePluckerLine(leftOC(Range(0,3), Range(0,1)), leftMInf);
      auto rightPluckerLine = computePluckerLine(rightOC(Range(0,3), Range(0,1)), rightMInf);

      auto intersection = computePluckerIntersection(leftPluckerLine, rightPluckerLine);
      cout << "Intersection: " << endl << intersection << endl;

      // Compare with OpenCV triangulation
      //The below code has been inspired from exercise from lecture 2 in Computer Vision.
      cout << "The OpenCV implmentation: " << endl;
      cv::Mat pnts3D(1, 1, CV_64FC4);
      cv::Mat cam0pnts(1, 1, CV_64FC2);
      cv::Mat cam1pnts(1, 1, CV_64FC2);
      cam0pnts.at<cv::Vec2d>(0)[0] = points[0].at<double>(0, 0);
      cam0pnts.at<cv::Vec2d>(0)[1] = points[0].at<double>(1, 0);
      cam1pnts.at<cv::Vec2d>(0)[0] = points[1].at<double>(0, 0);
      cam1pnts.at<cv::Vec2d>(0)[1] = points[1].at<double>(1, 0);
      triangulatePoints(leftProjectionMat, rightProjectionMat, cam0pnts, cam1pnts, pnts3D);
      std::cout << "OpenCV triangulation" << std::endl;
      std::cout << "Image points: " << cam0pnts << "\t" << cam1pnts << std::endl << std::endl;
      std::cout << "Triangulated point (normalized): " << std::endl << pnts3D / pnts3D.at<double>(3, 0) << std::endl << std::endl;

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
  sparseStereo.addNoiseToImages(0,0.1);
  sparseStereo.stereopsis();



  return 0;
}
