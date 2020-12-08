
//
//  sparseStereo.cpp
//  Robotics and Computer Vision
//
//  Created by Jens O. H. Iversen on 18/11/2020.
//  Inspired by:
//  1st semester Master's programme in Advanced Robotics Technology
//


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <rw/rw.hpp>

#define NUM_OBJECTS 3

double tmp_x=-1;
double tmp_y=-1;
bool compareVec(std::pair<cv::Point,double> &a, std::pair<cv::Point,double> &b){
  return a.second < b.second;
}

std::vector<rw::math::Transform3D<> > convertMatToTransform(std::vector<cv::Mat> points){
    std::vector<rw::math::Transform3D<> > transforms;
    for(unsigned int i = 0; i < points.size(); i++){
        rw::math::Transform3D<> point(rw::math::Vector3D<>(
                                                                     points[i].at<double>(0,0),
                                                                     points[i].at<double>(1,0),
                                                                     points[i].at<double>(2,0)),
                                            rw::math::RPY<>(0,rw::math::Deg2Rad * 180,0));
        
        transforms.push_back(point);
    }
    return transforms;
}

bool comparePoints(cv::Point &a, cv::Point &b){
  return a.x < b.x;
}

//Callback event for detecting mouse-click on images
void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        tmp_x = x;
        tmp_y = y;
    }
}

cv::Mat getMouseClick(std::string imageName, cv::Mat image){
    cv::imshow(imageName,image);
    cv::setMouseCallback(imageName,CallBackFunc,NULL);

    while(tmp_x < 0){
        cv::waitKey(100);
    }
    cv::Mat m(3, 1, CV_64F);
    m.at<double>(0, 0) = tmp_x;
    m.at<double>(1, 0) = tmp_y;
    m.at<double>(2, 0) = 1;

    tmp_x = -1;
    tmp_y = -1;
    return m;
}


cv::Mat convertMatrixToMat(Eigen::Matrix<double, 3, 4> matrix_){
  cv::Mat m(3,4, CV_64F);
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 4; j++){
      m.at<double>(i,j) = matrix_(i,j);
    }
  }
  return m;
}

std::array<cv::Mat, 2> splitProjectionMatrix(cv::Mat projectionMatrix){
  std::array<cv::Mat, 2> Pp;

  Pp[0] = projectionMatrix(cv::Range(0,3),cv::Range(0,3));
  Pp[1] = projectionMatrix(cv::Range(0,3),cv::Range(3,4));

  return Pp;
}

cv::Mat computeOpticalCenter(std::array<cv::Mat, 2> Pp){
  cv::Mat OC = -1.0 * Pp[0].inv(cv::DECOMP_SVD) * Pp[1];
  vconcat(OC,cv::Mat::ones(1,1,CV_64F),OC);
  return OC;
}

cv::Mat computeEpipole(cv::Mat projectionMatrix, cv::Mat OC){
  cv::Mat epipole = projectionMatrix * OC;
  return epipole;
}

cv::Mat computeFundamentalMatrix(cv::Mat epipole, cv::Mat projR, cv::Mat projL){
  cv::Mat epipoleCross = cv::Mat::zeros(3,3,CV_64F);
  epipoleCross.at<double>(0,1) = -epipole.at<double>(2);
  epipoleCross.at<double>(0,2) =  epipole.at<double>(1);
  epipoleCross.at<double>(1,0) =  epipole.at<double>(2);
  epipoleCross.at<double>(1,2) = -epipole.at<double>(0);
  epipoleCross.at<double>(2,0) = -epipole.at<double>(1);
  epipoleCross.at<double>(2,1) =  epipole.at<double>(0);

  cv::Mat psuedoProj = projL.inv(cv::DECOMP_SVD);
  return epipoleCross * projR * psuedoProj;

}

cv::Mat projectToInf(cv::Mat P, cv::Mat m){
  cv::Mat Minf = -1.0 * P.inv(cv::DECOMP_SVD) * m;
  return Minf;
}

std::array<cv::Mat, 2> computePluckerLine(cv::Mat M1, cv::Mat M2){
  std::array<cv::Mat, 2> munu;
  double n = cv::norm(M2);

  munu[0] = M1.cross(M2) / n;
  munu[1] = M2 / n;

  return munu;
}

cv::Mat computePluckerIntersection(std::array<cv::Mat, 2> plucker1, std::array<cv::Mat, 2> plucker2){
  cv::Mat mu1 = plucker1[0];
  cv::Mat nu1 = plucker1[1];
  cv::Mat mu2 = plucker2[0];
  cv::Mat nu2 = plucker2[1];

  double n2_M1 = norm(nu1.cross(nu2)) * norm(nu1.cross(nu2));
  double n2_M2 = norm(nu2.cross(nu1)) * norm(nu2.cross(nu1));

  cv::Mat M1 = ((( nu1.t() * nu2.cross(mu2) - (nu1.t() * nu2) * nu1.t() * nu2.cross(mu1) ) / n2_M1 ) * nu1.t()).t() + nu1.cross(mu1);
  cv::Mat M2 = ((( nu2.t() * nu1.cross(mu1) - (nu2.t() * nu1) * nu2.t() * nu1.cross(mu2) ) / n2_M2 ) * nu2.t()).t() + nu2.cross(mu2);

  return M1 + (M2 - M1) / 2;
}
