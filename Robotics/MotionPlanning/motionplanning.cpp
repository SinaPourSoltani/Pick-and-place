//
//  motionplanning.cpp
//  Robotics and Computer Vision
//
//  Created by Sina Pour Soltani and Jens O. H. Iversen on 30/11/2020.
//

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/Device.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/ProximityData.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <iostream>
#include <fstream>
#include <string>

#define DT 0.1
#define DT_VISU DT * 0.1
#define PLACE_CENTER_X 0.30
#define PLACE_CENTER_Y -0.50

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::pathplanning;
using namespace rws;

using rw::trajectory::QPath;
using rwlibs::pathplanners::RRTPlanner;
using rwlibs::proximitystrategies::ProximityStrategyFactory;

/*
 Point to Point interpolation
    - with parabolic blend
 RRT connect
*/

/*
 Set two points in 3D space. The interpolator creates a trajectory (a list of points) between the two points.
 For every dt in the duration of the trajectory, use the x, y and z position of the point in the trajectory
 to calculate joint values using inverse kinematics. Multiple solutions will be found.
 Pick the solution that changes the q values the least (sum of differences) (consider which q values could be neglected),
 since this solution will have a smooth transition from the previous point.
*/

/*
 Application:
 Recieve two homogeneous transformation matrices. Create the trajectory between them and
 return list of q-values.
 */

class MotionPlanner {
public:
    MotionPlanner(){};
    MotionPlanner(WorkCell::Ptr wc, SerialDevice::Ptr robot, State state) :
        wc(wc),
        robot(robot),
        state(state){};

    void setPickAndPlace(Transform3D<> newPick, string name, Transform3D<> newPlace){
        pick = newPick;
        place = newPlace;
        objectName = name;
        initialState = state;
        generatePoints();
    }

    // Linear interpolation
    InterpolatorTrajectory<Transform3D<> > linearlyInterpolate(vector<float> timeBetweenPoints, vector<float> blendFractions = {}){
        // Inspired from https://www.robwork.dk/apidoc/cpp/doxygen/classrw_1_1trajectory_1_1InterpolatorTrajectory.html
        vector<Transform3D<> > points = getPointTrajectorySequence();
        if(points.size() - 1 != timeBetweenPoints.size() || (!blendFractions.empty() && blendFractions.size() != timeBetweenPoints.size())){
          RW_THROW("linearlyInterpolate crash since input vectors doesn't match! <timeBetweenPoints> and <blendFractions> should have size one less than <points>.");
        }else if(!blendFractions.empty()){
          if(blendFractions[0] != 0.0){
            RW_THROW("First blend time must be zero!");
          }
        }

        InterpolatorTrajectory<Transform3D<> > trajectory;

        vector<LinearInterpolator<Transform3D<> >::Ptr> linearPoints;
        for(unsigned int point = 0; point < points.size() - 1; point++){
          linearPoints.push_back(ownedPtr(new LinearInterpolator<Transform3D<> >(points[point], points[point + 1], timeBetweenPoints[point])));
        }

        if(blendFractions.empty()){
          for(unsigned int i = 0; i < linearPoints.size(); i++){
            trajectory.add(linearPoints[i]);
          }
        }else{
          for(unsigned int i = 0; i < linearPoints.size(); i++){
            if(blendFractions[i] == 0){
                trajectory.add(linearPoints[i]);
            }else{
              ParabolicBlend<Transform3D<> >::Ptr blend = ownedPtr(new ParabolicBlend<Transform3D<> >(linearPoints[i - 1], linearPoints[i], timeBetweenPoints[i]*blendFractions[i]));
              trajectory.add(blend, linearPoints[i]);
            }
          }
        }
        return trajectory;
    }
    QPath trajectoryToQPath(InterpolatorTrajectory<Transform3D<> > trajectory){
        vector<Q> qvector;
        int counter = 0;
        for (double t = 0; t <= trajectory.duration(); t += DT) {
            Transform3D<> x = trajectory.x(t);
            vector<Q> qs = getJointConfigurations(x);
            if(qs.size() <= 0){
              cout << "Counter: " << ++counter << " / " << trajectory.duration()/DT << endl;
              continue;
            }
            //cout << "qs size: " << qs.size() << endl << "x: " << endl << x << endl << "Q: " << ((t==0) ? qs[0] : nearestJointConfiguration(qvector.back(),qs)) << endl << endl;
            qvector.push_back(((t==0) ? qs[0] : nearestJointConfiguration(qvector.back(),qs)));

        }
        QPath qpath(qvector);
        return qpath;
    }

    // RRT
    vector<QPath> RRTInterpolate(double extend) {
        vector<Transform3D<> > interpolationTransformsPoints = getPointTrajectorySequence(false);
        vector<Q> jointPoints;
        for(auto tf : interpolationTransformsPoints){
            vector<Q> jointConfigs = getJointConfigurations(tf);
            jointPoints.push_back(jointConfigs[0]); // TODO change to more optimal configuration rather than simply picking the first
        }
        return RRTConnect(jointPoints, extend);
    }


    void writeTimedStatePath(TimedStatePath path, string filePath){
      rw::loaders::PathLoader::storeTimedStatePath(*wc, path, filePath);
    }

    // Write to files
    void writeTrajectoryToFile(InterpolatorTrajectory<Transform3D<> > trajectory, string filePath){
        std::ofstream out(filePath);
        for (double t = 0; t <= trajectory.duration(); t += DT) {
            Transform3D<> x = trajectory.x(t);
            out << t << " " << x.P()(0) << " " << x.P()(1) << " " << x.P()(2) << endl;
        }
        out.close();
    }
    TimedStatePath writeQPathVectorToFile(vector<QPath> fullpath){
        state = initialState;
        TimedStatePath tStatePath;
        double time = 0;
        for(unsigned int i = 0; i < fullpath.size(); i++){
            for(unsigned int j = 0; j < fullpath[i].size(); j++){
                robot->setQ(fullpath[i][j], state);
                isPickingOrPlacing();
                tStatePath.push_back(TimedState(time, state));
                time += DT_VISU;
            }
        }
        return tStatePath;
        //rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, filePath);
    }
    TimedStatePath writeQPathToFile(QPath path, string filePath){
        cout << endl << "------------------------------------" << "writeQPathToFile called!" << "------------------------------------" << endl << endl;
        state = initialState;
        TimedStatePath tStatePath;
        double time = 0;
        for(unsigned int j = 0; j < path.size(); j++){
            robot->setQ(path[j], state);
            isPickingOrPlacing();
            tStatePath.push_back(TimedState(time, state));
            time += DT_VISU;
        }
        return tStatePath;
        //rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, filePath);
    }

    ~MotionPlanner(){};
protected:
    void generatePoints(){
        home = robot->frames().back()->wTf(state);
        pickApproach = Transform3D<>(Vector3D<>(pick.P()(0), pick.P()(1), home.P()(2)),
                                     home.R());

        pickDepart = Transform3D<>(pickApproach.P() + Vector3D<>(0, 0, pick.P()(2)),
                                   pick.R());

        placeApproach = Transform3D<>(Vector3D<>(place.P()(0), place.P()(1), home.P()(2)),
                                     place.R());

        placeDepart = Transform3D<>(placeApproach.P(),
                                    home.R());
    }
    vector<Transform3D<> > getPointTrajectorySequence(bool P2P = true){
        if(P2P){
          return {home, pickApproach, pick, pickDepart, home, placeApproach, place, placeDepart, home};
        }
        return {home, pick, place, home};
    }

    // collision free solution closes to current configuration
    // Joint configurations
    float distanceBetweenJoinsConfigurations(Q c1, Q c2){
      int num_joints = c1.size();
      float weight = 5;
      float distance = 0;
      for(int joint = 0; joint < num_joints; joint++){
        distance += weight-- * (c1[joint] - c2[joint]) * (c1[joint] - c2[joint]);
      }
      return distance;
    }
    Q nearestJointConfiguration(Q currentConfig, vector<Q> possibleConfigs){
      Q bestConfig = possibleConfigs[0];
      float bestDistance = distanceBetweenJoinsConfigurations(currentConfig, bestConfig);
      for(auto possibleConfig : possibleConfigs){
        float distance = distanceBetweenJoinsConfigurations(currentConfig, possibleConfig);
        if( distance < bestDistance){
          bestConfig = possibleConfig;
          bestDistance = distance;
        }
      }
      return bestConfig;
    }
    vector<Q> getJointConfigurations(Transform3D<> target, bool onlyCollisionFree = true) {

        // Get, make and print name of frames
        const string robotName = robot->getName();
        const string nameRobotBase = robotName + "." + "Base";
        const string nameRobotTcp = robotName + "." + "TCP";

        // Find frames and check for existence
        MovableFrame::Ptr tmpFrame = wc->findFrame<MovableFrame>("TargetFrame");
        tmpFrame->moveTo(target, wc->getWorldFrame(), state);
        Frame* frameTarget = wc->findFrame("TargetFrame");
        Frame* frameTcp = wc->findFrame("GraspTCP");
        Frame* frameRobotBase = wc->findFrame(nameRobotBase);
        Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
        if(frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
        {
            cout << " ALL FRAMES NOT FOUND:" << std::endl;
            cout << " Found \"" << "GraspTCP" << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << endl;
            cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << endl;
            cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << endl;
        }

        isPickingOrPlacing(target);
        if (compareTransforms(target, home)) {
            cout << "True home" << endl;
            Q home(6,-0.00403171, -1.06205, -1.68285, -1.96749, 1.5708, -0.00403171);
            return {home};
        }

        // Make "helper" transformations
        Transform3D<> frameBaseTGoal = Kinematics::frameTframe(frameRobotBase, frameTarget, state);
        //Transform3D<> frameTcpTRobotTcp = Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

        // get grasp frame in robot tool frame
        Transform3D<> targetAt = frameBaseTGoal * Transform3D<>(Vector3D<>(0,0,-0.07191),RPY<>(0,0,0));//* frameTcpTRobotTcp;

        rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
        vector<Q> solutions = closedFormSovler->solve(targetAt, state);

        if(!onlyCollisionFree) {
            return solutions;
        } else {
            vector<Q> collisionFreeSolutions;
            CollisionDetector::Ptr detector = ownedPtr(new CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

            for(unsigned int i = 0; i < solutions.size(); i++){
                // set the robot in that configuration and check if it is in collision
                robot->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) ){
                    collisionFreeSolutions.push_back(solutions[i]);
                }
            }

            return collisionFreeSolutions;
        }
    }

    // RRT helper functions
    vector<QPath> RRTConnect(vector<Q> interpolationJointsPoints, double extend){
        // State tmpState = state;
        vector<QPath> totalPath;
        for (unsigned int i = 0; i < interpolationJointsPoints.size() - 1; i++) {
            Q from(interpolationJointsPoints[i]);
            Q to(interpolationJointsPoints[i + 1]);
            totalPath.push_back(RRTConnectQtoQ(from, to, extend));
        }
        // state = tmpState;
        return totalPath;
    }
    QPath RRTConnectQtoQ(Q from, Q to, double extend){
        // Inspired from RoVi sample plugin
        // and https://www.robwork.dk/manual/motionplanning/
        Math::seed();
        CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
        PlannerConstraint constraint = PlannerConstraint::make(&detector,robot,state);

        QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(robot), constraint.getQConstraintPtr());
        QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
        cout << "Extend: " << extend << endl;

        const QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner (constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

        ProximityData pdata;
        robot->setQ(from,state);
        isPickingOrPlacing();
        if (detector.inCollision (state, pdata))
            RW_THROW ("Initial configuration in collision! can not plan a path.");
        robot->setQ (to, state);
        if (detector.inCollision (state, pdata))
            RW_THROW ("Final configuration in collision! can not plan a path.");

        QPath result;
        if (planner->query (from, to, result)) {
            std::cout << "Planned path with " << result.size ();
            std::cout << " configurations" << std::endl;
        }else{
          RW_THROW("Could not find RRT path between the two points");
        }


        return result;
    }

    bool compareTransforms(Transform3D<> t1, Transform3D<> t2){
        double threshold = 0.015 * 1000;
        for(int i = 0; i < 3; i++){
            int p1 = round(t1.P()(i) * 1000);
            int p2 = round(t2.P()(i) * 1000);
            //cout << endl << "t1: " << p1 << " - t2: " << p2;
            if( p1 != p2) {
              if(i == 2 && p1 <= p2 + threshold && p1 >= p2 - threshold){
                cout << "EQUAL TRANSFORMS" << endl;
                return true;
              }
                //cout << " !=";
                return false;
            } else {
                //cout << " ==";
            }

        }
        cout << "EQUAL TRANSFORMS" << endl;
        return true;
    }
    void isPickingOrPlacing(Transform3D<> H = Transform3D<>::identity()){
        Transform3D<> tcpH = H;
        Frame * tcpFrame = wc->findFrame("WSG50.Base");
        Frame * objectFrame = wc->findFrame(objectName);
        if (H == Transform3D<>::identity()){
            tcpH = tcpFrame->wTf(state);

        }
        if(compareTransforms(tcpH, pick)) {
            cout << "picked" << endl;
            Kinematics::gripFrame(objectFrame, tcpFrame, state);
        } else if (compareTransforms(tcpH, place)) {
            cout << "placed" << endl;
            Kinematics::gripFrame(objectFrame, wc->getWorldFrame(), state);
        }
    }

    WorkCell::Ptr wc;
    SerialDevice::Ptr robot;
    State state, initialState;
    Transform3D<> home, pick, pickApproach, pickDepart, place, placeApproach, placeDepart;
    string objectName;
};
