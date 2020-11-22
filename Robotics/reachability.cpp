//
//  reachability.cpp
//  Robotics and Computer Vision
//
//  Created by Sina Pour Soltani and Jens O. H. Iversen on 30/10/2020.
//  Inspired by solution to programming exercise 5 from Robotics course from
//  1st semester Master's programme in Advanced Robotics Technology
//

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

vector<Q> getConfigurations(const string nameGoal, const string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const string robotName = robot->getName();
    const string nameRobotBase = robotName + "." + "Base";
    const string nameRobotTcp = robotName + "." + "TCP";
    
    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }
    
    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);
    
    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;
    
    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

#define MIN_Y_WORKAREA -0.525
#define MAX_Y_WORKAREA 0.175
#define MIN_X_WORKAREA -0.325
#define MAX_X_WORKAREA 0.325

#define MAX_X_PLACEAREA 0.125
#define MAX_Y_PLACEAREA -0.325

#define STEP_RES 3
#define STEP_SIZE_X (MAX_X_WORKAREA - MIN_X_WORKAREA) / STEP_RES
#define STEP_SIZE_Y (MAX_Y_WORKAREA - MIN_Y_WORKAREA) / STEP_RES

bool isInsideWorkArea(double x, double y){
    return !(x > MAX_X_PLACEAREA && y < MAX_Y_PLACEAREA);
}

vector<Vector3D<>> getBaseFramePositions(){
    vector<Vector3D<>> base_positions;
    
    for (double y=MIN_Y_WORKAREA; y <= MAX_Y_WORKAREA; y += STEP_SIZE_Y) {
        for (double x=MIN_X_WORKAREA; x <= MAX_X_WORKAREA; x += STEP_SIZE_X) {
            if (isInsideWorkArea(x,y)) {
                Vector3D<> pos(x, y, 0.01);
                base_positions.push_back(pos);
            }
        }
    }
    return base_positions;
}



int main(int argc, char** argv)
{
    // how to call the script
    if(argc < 4)
    {
        cout << "Usage: " << argv[0] << " <scene> <object> <path/to/save/rw.play>" << endl;
        return 0;
    }
    
    string scene = string(argv[1]);
    string object = string(argv[2]);
    string rwplayPath = string(argv[3]);
    
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
    
    
    // setup collision detector
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    
    State state = wc->getDefaultState();
    vector<rw::math::Q> collisionFreeSolutions;
    
    Rotation3D<> objectRotInv = inverse(objectFrame->getTransform(state).R());
    
    TimedStatePath tStatePath;
    double time = 0;
    
    vector<Vector3D<>> base_positions = getBaseFramePositions();
    Rotation3D<> base_rot = base_frame->getTransform(state).R();
    
    for(auto base_pos : base_positions) {
        cout << "Base: " << base_pos << endl;
        base_frame->moveTo(Transform3D<>(base_pos, base_rot),state);
        
        for(double rollAngle = 0; rollAngle <= 180.0; rollAngle += 10.0)
        {
            cout << "RollAngle: " << rollAngle << endl;
            
            for(double pitchAngle = 90; pitchAngle <= 270; pitchAngle += 5.0)
            {
                
                objectGraspFrame->moveTo(Transform3D<>(
                                                       Vector3D<>(
                                                                  tableFrame->getTransform(state).P() + objectFrame->getTransform(state).P()),
                                                       RPY<>(rollAngle * Deg2Rad, 0, pitchAngle * Deg2Rad).toRotation3D()
                                                       )
                                         , state);
                
                vector<rw::math::Q> solutions = getConfigurations(graspTarget, "GraspTCP",robot, wc, state);
                
                for(unsigned int i=0; i<solutions.size(); i++){
                    // set the robot in that configuration and check if it is in collision
                    robot->setQ(solutions[i], state);
                    if( !detector->inCollision(state,NULL,true) ){
                        
                        collisionFreeSolutions.push_back(solutions[i]); // save it
                        
                        robot->setQ(solutions[i], state);
                        tStatePath.push_back(TimedState(time,state));
                        time += 0.01;
                        
                        //break; // we only need one
                    }
                }
            }
        }
    }
    
    
    cout << "Current position of the robot vs object to be grasped has: "
    << collisionFreeSolutions.size()
    << " collision-free inverse kinematics solutions!" << std::endl;
    
    // visualize them
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplayPath);
    
    return 0;
    
}
