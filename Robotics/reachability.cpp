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

std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

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



int main(int argc, char** argv)
{
    // how to call the script
    if(argc < 4)
    {
        cout << "Usage: " << argv[0] << " <scene> <robot> <object>" << endl;
        return 0;
    }

    // load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(argv[1]);
    if(wc == NULL){
        RW_THROW("COULD NOT LOAD scene... check path!");
        return -1;
    }

    // finding serial device
    rw::models::SerialDevice::Ptr robot = wc->findDevice<rw::models::SerialDevice>(argv[2]);
    if(robot==NULL)
    {
        RW_THROW("COULD not find device robot... check model");
        return -1;
    }

    // find relevant frames
    rw::kinematics::MovableFrame::Ptr objectFrame = wc->findFrame<rw::kinematics::MovableFrame>(argv[3]);
    if(objectFrame==NULL)
    {
        RW_THROW("COULD not find movable object frame... check model");
        return -1;
    }

    //Getting Table frames
    rw::kinematics::FixedFrame::Ptr tableFrame = wc->findFrame<rw::kinematics::FixedFrame>("Table");
    if(tableFrame==NULL)
    {
        RW_THROW("COULD not find Table frame... check model");
        return -1;
    }


    // setup collision detector
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


    State state = wc->getDefaultState();
    vector<rw::math::Q> collisionFreeSolutions;

    rw::math::Rotation3D<> objectRotInv = inverse(objectFrame->getTransform(state).R());


    for(double rollAngle = 0; rollAngle < 360.0; rollAngle += 5.0)
    {
	      cout << "RollAngle: " << rollAngle << endl;


        //Ved 180 går den under objektet.
        for(double pitchAngle = 0; pitchAngle < 180.0; pitchAngle += 5.0) // TODO check if 180 is enough
        {



            objectFrame->moveTo(
              rw::math::Transform3D<>(
                rw::math::Vector3D<>(tableFrame->getTransform(state).P() + objectFrame->getTransform(state).P()),
                (rw::math::RPY<>(0,180*rw::math::Deg2Rad,90*rw::math::Deg2Rad).toRotation3D()) *
                 (rw::math::RPY<>(pitchAngle*rw::math::Deg2Rad,rollAngle*rw::math::Deg2Rad,0).toRotation3D())
                )
              , state);

            string graspTarget = string(argv[3]) + "GraspTarget";
            vector<rw::math::Q> solutions = getConfigurations(graspTarget, "GraspTCP",robot, wc, state);

            for(unsigned int i=0; i<solutions.size(); i++){
                // set the robot in that configuration and check if it is in collision
                robot->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) ){
                    collisionFreeSolutions.push_back(solutions[i]); // save it
                    break; // we only need one
                }
            }
        }
    }

    cout << "Current position of the robot vs object to be grasped has: "
    << collisionFreeSolutions.size()
    << " collision-free inverse kinematics solutions!" << std::endl;

    // visualize them
    TimedStatePath tStatePath;
    double time=0;
    for(unsigned int i=0; i<collisionFreeSolutions.size(); i++){
        robot->setQ(collisionFreeSolutions[i], state);
        tStatePath.push_back(TimedState(time,state));
        time+=0.01;
    }

    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../Project_WorkCell/visu.rwplay");

    return 0;

}
