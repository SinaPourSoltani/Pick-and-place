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
#include <fstream>
#include <string>

#define MIN_Y_WORKAREA -0.525
#define MAX_Y_WORKAREA 0.175
#define MIN_X_WORKAREA -0.325
#define MAX_X_WORKAREA 0.325

#define MAX_X_PLACEAREA 0.125
#define MAX_Y_PLACEAREA -0.325

#define STEP_RES 20
#define STEP_SIZE_X (MAX_X_WORKAREA - MIN_X_WORKAREA) / STEP_RES
#define STEP_SIZE_Y (MAX_Y_WORKAREA - MIN_Y_WORKAREA) / STEP_RES

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;

vector<Q> getConfigurations(const string nameGoal, const string nameTcp, SerialDevice::Ptr robot, WorkCell::Ptr wc, State state)
{
    // Get, make and print name of frames
    const string robotName = robot->getName();
    const string nameRobotBase = robotName + "." + "Base";
    const string nameRobotTcp = robotName + "." + "TCP";
    
    // Find frames and check for existence
    Frame* frameGoal = wc->findFrame(nameGoal);
    Frame* frameTcp = wc->findFrame(nameTcp);
    Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        cout << " ALL FRAMES NOT FOUND:" << std::endl;
        cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << endl;
        cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << endl;
    }
    
    // Make "helper" transformations
    Transform3D<> frameBaseTGoal = Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    Transform3D<> frameTcpTRobotTcp = Kinematics::frameTframe(frameTcp, frameRobotTcp, state);
    
    // get grasp frame in robot tool frame
    Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;
    
    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

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
    
    ofstream outputFile;
    outputFile.open(outputFilePath);
    
    outputFile << scene << endl;
    outputFile << object << endl;
    outputFile << "x y #collisionFreeSolutions"  << endl;
    
    // setup collision detector
    CollisionDetector::Ptr detector = ownedPtr(new CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    
    State state = wc->getDefaultState();
    vector<Q> collisionFreeSolutions;
    
    Rotation3D<> objectRotInv = inverse(objectFrame->getTransform(state).R());
    
    TimedStatePath tStatePath;
    double time = 0;
    
    vector<Vector3D<>> base_positions = getBaseFramePositions();
    Rotation3D<> base_rot = base_frame->getTransform(state).R();
    
    int num_collision_free_solutions = 0;
    int index = 0;
    for(auto base_pos : base_positions) {
        cout << ++index << "/" << base_positions.size() << endl;
        cout << "Base: " << base_pos << endl;
        base_frame->moveTo(Transform3D<>(base_pos, base_rot),state);
        
        num_collision_free_solutions = 0;
        for(double rollAngle = 0; rollAngle <= 180.0; rollAngle += 5.0)
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
                        num_collision_free_solutions++;
                        robot->setQ(solutions[i], state);
                        tStatePath.push_back(TimedState(time,state));
                        time += 0.01;
                        
                        //break; // we only need one
                    }
                }
            }
        }
        // write to file
        // x y and number of collision free solutions
        // new line for each positions
        outputFile << base_pos[0] << " " << base_pos[1] << " " << num_collision_free_solutions << endl;
    }
    
    outputFile.close();
    
    cout << "Current position of the robot vs object to be grasped has: "
    << collisionFreeSolutions.size()
    << " collision-free inverse kinematics solutions!" << std::endl;
    
    // visualize them
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplayPath);
    
    return 0;
    
}
