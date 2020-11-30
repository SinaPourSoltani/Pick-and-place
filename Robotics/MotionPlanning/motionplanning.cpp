//
//  motionplanning.cpp
//  Robotics and Computer Vision
//
//  Created by Sina Pour Soltani and Jens O. H. Iversen on 30/11/2020.
//

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <fstream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;

int main(int argc, char**argv){
    // how to call the script
    if(argc < 5) {
        cout << "Usage: " << argv[0] << " <scene> <object> <path/to/save/rw.play> <path/to/save/collisions_heatmap.txt>" << endl;
        return 0;
    }
    
    string scene = string(argv[1]);
    string object = string(argv[2]);
    string rwplayPath = string(argv[3]);
    string outputFilePath = string(argv[4]);
    
    // load workcell
    WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(scene);
    if(wc == NULL) {
        RW_THROW("COULD NOT LOAD scene... check path!");
        return -1;
    }
    
    // find robot base frame
    MovableFrame::Ptr base_frame = wc->findFrame<MovableFrame>("URReference");
    if(base_frame==NULL) {
        RW_THROW("COULD not find robot base... check model");
        return -1;
    }
    
    // find serial device
    SerialDevice::Ptr robot = wc->findDevice<SerialDevice>("UR-6-85-5-A");
    if(robot==NULL) {
        RW_THROW("COULD not find device robot... check model");
        return -1;
    }
    
    // find relevant frames
    MovableFrame::Ptr objectFrame = wc->findFrame<MovableFrame>(object);
    if(objectFrame==NULL) {
        RW_THROW("COULD not find movable object frame... check model");
        return -1;
    }
    
    string graspTarget = "GraspTarget" + object;
    MovableFrame::Ptr objectGraspFrame = wc->findFrame<MovableFrame>(graspTarget);
    if(objectGraspFrame==NULL) {
        RW_THROW("COULD not find movable objectGrasp frame... check model");
        return -1;
    }
    
    //Getting Table frames
    FixedFrame::Ptr tableFrame = wc->findFrame<FixedFrame>("Table");
    if(tableFrame==NULL) {
        RW_THROW("COULD not find Table frame... check model");
        return -1;
    }
    
    
    
}
