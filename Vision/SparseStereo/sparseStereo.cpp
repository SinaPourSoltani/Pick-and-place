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
#include <thread>

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
#define SHOWIMAGES true

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
    string pathRightImage = rightImageStr;
    string pathLeftImage = leftImageStr;
    string leftImageNoiseStr = "left_image_noise.png";
    string rightImageNoiseStr = "right_image_noise.png";
    string pathLeftImageNoise = leftImageNoiseStr;
    string pathRightImageNoise = rightImageNoiseStr;

    double leftFovy, rightFovy;
    int leftWidth, leftHeight, rightWidth, rightHeight;

    State state;

    Eigen::Matrix<double, 3, 4> leftProjectionMatrix;
    Eigen::Matrix<double, 3, 4> rightProjectionMatrix;



    //This funciton is inspired from Sina's work
    void addGaussianNoiseToImage(string image_, string output, int mean = 0, double variance = 0.1){
        cout << "\nAdding noise to " << image_ << endl;
        Mat img = imread(image_, 1);
        Scalar a = cv::mean(img); //Must have cv:: before or it wont compile. I think i takes another mean function from RobWorkStudio or something.
        float power = (a.val[0] + a.val[1] + a.val[2]) / 3;
        Mat noise(img.size(), img.type());
        randn(noise, mean, power * variance);
        addWeighted(img, 1.0, noise, 1.0, 0.0, img);
        imwrite(output, img);
    }

    void addSaltAndPepperNoiseToImage(string image_, string output, float amount){
      if(amount < 0.0 || amount > 1.0){
        throw("The % of salt and pepper noise has to be a float between 0.0 and 1.0");
      }
      Mat img = imread(image_, 1);
      float randomNumber = 0.0;
      int rgbPicker = 0;
      for(int row = 0; row < img.rows; row++){
        for(int col = 0; col < img.cols; col++){
          randomNumber = ((float)rand() / (RAND_MAX));
          if(randomNumber <= amount){
            rgbPicker = rand() % 3;
            switch (rgbPicker) {
              case 0:
                img.at<Vec3b>(row,col)[0] = 255;
                img.at<Vec3b>(row,col)[1] = 0;
                img.at<Vec3b>(row,col)[2] = 0;
                break;
              case 1:
                img.at<Vec3b>(row,col)[0] = 0;
                img.at<Vec3b>(row,col)[1] = 255;
                img.at<Vec3b>(row,col)[2] = 0;
                break;
              default:
                img.at<Vec3b>(row,col)[0] = 0;
                img.at<Vec3b>(row,col)[1] = 0;
                img.at<Vec3b>(row,col)[2] = 255;
                break;
            }
          }
        }
      }
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

    /*Finds the centers of the cylinders. Does this by finding contours
     * in the image. Sorting the contours by area and taking the three smallest
     * which is the cylinders. Then it sorts the three center points, from the
     * contour of the cylinders, by their x coordinate.
     */
    array<Mat,NUM_OBJECTS> findCenter(string imagePath, bool showImage = false){
        array<Mat,3> points;
        Mat src = imread(imagePath, IMREAD_COLOR);
        Mat img = src.clone();
        cvtColor(src, img, COLOR_RGBA2GRAY, 0);
        threshold(img, img, 160, 255, THRESH_BINARY_INV);
        imwrite("Threshold.png", img);
        vector<vector<Point>> contours;
        findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<pair<Point,double>> coordsArea;
        for(unsigned int i = 0; i < contours.size(); i++){
            Moments m = moments(contours[i], false);
            double area = contourArea(contours[i]);
            if(area <= 100){ //If area is zero we don't consider the contour.
                continue;
            }
            pair<Point,double> p;
            //The equation under is found here: https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
            p.first = Point(m.m10 / m.m00, m.m01 / m.m00);
            p.second = area;
            coordsArea.push_back(p);
        }
        //Sorting so the three first is the cylinders.
        sort(coordsArea.begin(), coordsArea.end(), compareVec);
        vector<Point> pointsVec;
        for(int i = 0; i < NUM_OBJECTS; i++){
            pointsVec.push_back(coordsArea[i].first);
        }
        sort(pointsVec.begin(), pointsVec.end(), comparePoints);
        for(int i = 0; i < NUM_OBJECTS; i++){
            Mat point(3,1, CV_64F);
            point.at<double>(0,0) = pointsVec[i].x;
            point.at<double>(1,0) = pointsVec[i].y;
            point.at<double>(2,0) = 1.0;
            points[i] = point;
        }

        if(showImage){
            circle(src, pointsVec[0], 2, Scalar(0, 255, 255), -1);
            circle(src, pointsVec[1], 2, Scalar(0, 255, 255), -1);
            circle(src, pointsVec[2], 2, Scalar(0, 255, 255), -1);
            imshow(imagePath, src);
            waitKey(0);
            //destroyAllWindows();
        }
        return points;
    }

public:
    SparseStereo(){};
    SparseStereo(string sceneFilePath){
        scene = sceneFilePath;
        leftCam = "Camera_Left";
        rightCam = "Camera_Right";
        loadWorkcellAndCameras();

        RobWorkStudioApp app("");

        thread t([&](){
            while (!app.isRunning()) {
                TimerUtil::sleepMs(1000);
                cout << "Waiting for launch.." << endl;
            }
            RobWorkStudio* rwstudio = app.getRobWorkStudio();
            rwstudio->postOpenWorkCell(scene);
            TimerUtil::sleepMs(2000);

            takeImages(rwstudio);

            app.close();
        });
        t.detach();
        app.run();
    }

    void loadWorkcellAndCameras(){
        wc = WorkCellLoader::Factory::load(scene);
        if(wc.isNull()){RW_THROW("WorkCell could not be loaded.");}

        leftCamera = wc->findFrame(leftCam);
        if(leftCamera == nullptr){RW_THROW("Left camera frame could not be found.");}

        rightCamera = wc->findFrame(rightCam);
        if(rightCamera == nullptr){RW_THROW("Right camera frame could not be found.");}

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

    void takeImages(RobWorkStudio* rwstudio){
        //Setting up for leftCamera
        const SceneViewer::Ptr leftGldrawer = rwstudio->getView()->getSceneViewer();
        const GLFrameGrabber::Ptr leftFramegrabber = ownedPtr(new GLFrameGrabber(leftWidth,leftHeight,leftFovy));
        leftFramegrabber->init(leftGldrawer);
        SimulatedCamera::Ptr leftSimcam = ownedPtr(new SimulatedCamera("SimulatedCamera", leftFovy, leftCamera, leftFramegrabber));
        leftSimcam->setFrameRate(100);
        leftSimcam->initialize();
        leftSimcam->start();
        leftSimcam->acquire();


        static const double dt = 0.001;
        const Simulator::UpdateInfo leftInfo(dt);
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

        const Simulator::UpdateInfo rightInfo(dt);
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
    }

    void addGaussianNoiseToImages(int mean = 0, double variance = 0.1){
        addGaussianNoiseToImage(pathLeftImage, pathLeftImageNoise, mean, variance);
        addGaussianNoiseToImage(pathRightImage, pathRightImageNoise, mean, variance);
    }
    void addSaltAndPepperNoiseToImages(float amount = 0.10){
      addSaltAndPepperNoiseToImage(pathLeftImage, pathLeftImageNoise, amount);
      addSaltAndPepperNoiseToImage(pathRightImage, pathRightImageNoise, amount);
    }

    //This function is temp until feature detection is applied.
    array<Mat,2> openImages(){
        array<Mat,2> points;
        Mat leftImg = imread(pathLeftImageNoise, IMREAD_COLOR);
        points[0] = getMouseClick("Left image.", leftImg);
        //cout << "Left point: \n" << points[0] << endl;

        Mat rightImg = imread(pathRightImageNoise, IMREAD_COLOR);
        points[1] = getMouseClick("Right image.", rightImg);
        //cout << "Right point: \n" << points[1] << endl;

        return points;
    }

    vector<Mat> stereopsis(){
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
        //auto points = openImages();
        auto correspondingPoints = findCenters();
        vector<Mat> points3D;
        for(int i = 0; i < NUM_OBJECTS; i++){
            array<Mat,2> points;
            points[0] = correspondingPoints[i].first;
            points[1] = correspondingPoints[i].second;
            //cout << "Point no: " << i+1 << endl;


            auto leftMInf = projectToInf(leftPp[0], points[0]);
            auto rightMInf = projectToInf(rightPp[0], points[1]);

            auto leftPluckerLine = computePluckerLine(leftOC(Range(0,3), Range(0,1)), leftMInf);
            auto rightPluckerLine = computePluckerLine(rightOC(Range(0,3), Range(0,1)), rightMInf);

            auto intersection = computePluckerIntersection(leftPluckerLine, rightPluckerLine);
            //cout << "Intersection: " << endl << intersection << endl;
            points3D.push_back(intersection);

            // Compare with OpenCV triangulation
            //The below code has been inspired from exercise from lecture 2 in Computer Vision.
            /*
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
             */
        }
        return points3D;
    }

    //Finds the centers of the cylinders.

    array<pair<Mat, Mat>,3> findCenters(){
        auto leftPoints = findCenter(pathLeftImageNoise, SHOWIMAGES);
        auto rightPoints = findCenter(pathRightImageNoise, SHOWIMAGES);
        array<pair<Mat, Mat>,3> correspondingPoints;
        for(int i = 0; i < NUM_OBJECTS; i++){
            pair<Mat, Mat> p;
            p.first = leftPoints[i];
            p.second = rightPoints[i];
            correspondingPoints[i] = p;
        }
        return correspondingPoints;
    }

    ~SparseStereo(){};
};
