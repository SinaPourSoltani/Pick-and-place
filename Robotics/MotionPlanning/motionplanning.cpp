//
//  motionplanning.cpp
//  Robotics and Computer Vision
//
//  Created by Sina Pour Soltani and Jens O. H. Iversen on 30/11/2020.
//

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
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
    MotionPlanner(WorkCell::Ptr wc, SerialDevice::Ptr robot, Transform3D<> pick, Transform3D<> place, State state) :
        wc(wc),
        robot(robot),
        pick(pick),
        place(place),
        state(state) {
            generatePoints();
        };

    // collision free solution closes to current configuration
    float distanceBetweenJoinsConfigurations(Q c1, Q c2){
      int num_joints = c1.size();
      float weight = 5;
      float distance = 0;
      for(unsigned int joint = 0; joint < num_joints; joint++){
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
            Q home(6,-0.00403171, -0.991591, -1.99674, -1.72721, 1.5708, 0.00225147);
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
    InterpolatorTrajectory<Transform3D<> > linearlyInterpolate(vector<float> timeBetweenPoints, vector<float> blendFractions = {}/*bool withBlend = false*/){
        // Inspired from https://www.robwork.dk/apidoc/cpp/doxygen/classrw_1_1trajectory_1_1InterpolatorTrajectory.html#a8603623d1793b8b55400f60de893d62d
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
        for (double t = 0; t <= trajectory.duration(); t += DT) {
            Transform3D<> x = trajectory.x(t);
            vector<Q> qs = getJointConfigurations(x);
            qvector.push_back(((t==0) ? qs[0] : nearestJointConfiguration(qvector.back(),qs)));

        }
        QPath qpath(qvector);
        return qpath;
    }

    vector<QPath> RRTInterpolate(double extend) {
        vector<Transform3D<> > interpolationTransformsPoints = getPointTrajectorySequence(false);
        vector<Q> jointPoints;
        for(auto tf : interpolationTransformsPoints){
            vector<Q> jointConfigs = getJointConfigurations(tf);
            jointPoints.push_back(jointConfigs[0]); // TODO change to more optimal configuration rather than simply picking the first
        }
        return RRTConnect(jointPoints, extend);
    }
    vector<QPath> RRTConnect(vector<Q> interpolationJointsPoints, double extend){
        vector<QPath> totalPath;
        for (unsigned int i = 0; i < interpolationJointsPoints.size() - 1; i++) {
            Q from(interpolationJointsPoints[i]);
            Q to(interpolationJointsPoints[i + 1]);
            totalPath.push_back(RRTConnectQtoQ(from, to, extend));
        }
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

    void writeTrajectoryToFile(InterpolatorTrajectory<Transform3D<> > trajectory, string filePath){
        std::ofstream out(filePath);
        for (double t = 0; t <= trajectory.duration(); t += DT) {
            Transform3D<> x = trajectory.x(t);
            out << t << " " << x.P()(0) << " " << x.P()(1) << " " << x.P()(2) << endl;
        }
        out.close();
    }
    void writeQPathVectorToFile(vector<QPath> fullpath, string filePath){
        state = wc->getDefaultState();
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
        rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, filePath);
    }
    void writeQPathToFile(QPath path, string filePath){
        cout << endl << endl << "------------------------------------" << "writeQPathToFile called!" << "------------------------------------" << endl << endl;
        state = wc->getDefaultState();
        TimedStatePath tStatePath;
        double time = 0;
        for(unsigned int j = 0; j < path.size(); j++){
            robot->setQ(path[j], state);
            isPickingOrPlacing();
            tStatePath.push_back(TimedState(time, state));
            time += DT_VISU;
        }
        rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, filePath);
    }

    ~MotionPlanner(){};
protected:
    void generatePoints(){
        home = robot->frames().back()->wTf(state);
        pickApproach = Transform3D<>(Vector3D<>(pick.P()(0), pick.P()(1), home.P()(2)),
                                     pick.R());

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
        Frame * objectFrame = wc->findFrame("CylinderRed");
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
    State state;
    Transform3D<> home, pick, pickApproach, pickDepart, place, placeApproach, placeDepart;
};


int main(int argc, char**argv){
    // how to call the script
    if(argc < 2) {
        cout << "Usage: " << argv[0] << " <scene> <path/to/save/trajectory.dat> <path/to/save/qpath.rwplay>" << endl;
        return 0;
    }

    string scene = string(argv[1]);
    string trajectoryFilePath = "traj.dat"; //string(argv[2]);
    string qpathFilePath = "qconfig.rwplay"; //string(argv[3]);

    // load workcell
    WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(scene);
    if(wc == NULL) {
        RW_THROW("COULD NOT LOAD scene... check path!");
        return -1;
    }

    // find serial device
    SerialDevice::Ptr robot = wc->findDevice<SerialDevice>("UR-6-85-5-A");
    if(robot==NULL) {
        RW_THROW("COULD not find device robot... check model");
        return -1;
    }

    // home                 : defined in the serial device
    // approach to target   : target pos with home z
    // grasp target         : one of the two inputs
    // back to appraoch     : target pos with home z + object height
    // home                 : defined in the serial device
    // approach to goal     : goal pos with home z
    // deliver target       : one of the two inputs
    // back to apprach      : goal pos with home z
    // home                 : defined in the serial device

    Math::seed();

    State state = wc->getDefaultState();

    Transform3D<> pick(Vector3D<>(0.25, 0.474, 0.191), RPY<>(0,Deg2Rad * 180,0));
    Transform3D<> place(Vector3D<>(PLACE_CENTER_X, PLACE_CENTER_Y, 0.191), RPY<>(0,Deg2Rad * 180,0));

    MotionPlanner mp(wc, robot, pick, place, state);
    vector<float> timeBetweenPoints = {1, 1, 1, 1, 1, 1, 1, 1};
    vector<float> blendFractions = {0.0, 0.5, 0.0, 0.5, 0.5, 0.5, 0.0, 0.5};
    InterpolatorTrajectory<Transform3D<> > trajectory = mp.linearlyInterpolate(timeBetweenPoints, blendFractions);
    QPath lipath = mp.trajectoryToQPath(trajectory);
    //mp.writeTrajectoryToFile(trajectory, trajectoryFilePath);
    mp.writeQPathToFile(lipath, qpathFilePath);

    // TODO Attach object
    // choose closest joint values

    //vector<QPath> rrtpath = mp.RRTInterpolate(0.05);
    //mp.writeQPathVectorToFile(rrtpath, "qconfig.rwplay");
}
