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

    // collission free solution closes to current configuration

    vector<Q> getJointConfigurations(Transform3D<> target, bool onlyCollissionFree = true) {

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

        // Make "helper" transformations
        Transform3D<> frameBaseTGoal = Kinematics::frameTframe(frameRobotBase, frameTarget, state);
        Transform3D<> frameTcpTRobotTcp = Kinematics::frameTframe(frameTcp, frameRobotTcp, state);
        cout << "frameBaseTGoal: " << endl << frameBaseTGoal << endl;
        cout << "frameTcpTRobotTcp: " << endl << frameTcpTRobotTcp << endl;

        // get grasp frame in robot tool frame
        Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

        cout << "TargetAt: " << endl << targetAt << endl;
        cout << "Target: " << endl << target << endl << endl;


        rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
        vector<Q> solutions = closedFormSovler->solve(targetAt, state);

        if(!onlyCollissionFree) {
            return solutions;
        } else {
            vector<Q> collisionFreeSolutions;
            CollisionDetector::Ptr detector = ownedPtr(new CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

            for(int i = 0; i < solutions.size(); i++){
                // set the robot in that configuration and check if it is in collision
                robot->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) ){
                    collisionFreeSolutions.push_back(solutions[i]);
                }
            }

            return collisionFreeSolutions;
        }
    }

    InterpolatorTrajectory<Transform3D<> > linearlyInterpolate(bool withBlend = false){
        // Inspired from https://www.robwork.dk/apidoc/cpp/doxygen/classrw_1_1trajectory_1_1InterpolatorTrajectory.html#a8603623d1793b8b55400f60de893d62d
        LinearInterpolator<Transform3D<> >::Ptr hm2pka = ownedPtr(new LinearInterpolator<Transform3D<> >(home, pickApproach, 1));
        LinearInterpolator<Transform3D<> >::Ptr pka2pk = ownedPtr(new LinearInterpolator<Transform3D<> >(pickApproach, pick, 1));
        LinearInterpolator<Transform3D<> >::Ptr pk2pkd = ownedPtr(new LinearInterpolator<Transform3D<> >(pick, pickDepart, 1));
        LinearInterpolator<Transform3D<> >::Ptr pkd2hm = ownedPtr(new LinearInterpolator<Transform3D<> >(pickDepart, home, 1));

        LinearInterpolator<Transform3D<> >::Ptr hm2pla = ownedPtr(new LinearInterpolator<Transform3D<> >(home, placeApproach, 1));
        LinearInterpolator<Transform3D<> >::Ptr pla2pl = ownedPtr(new LinearInterpolator<Transform3D<> >(placeApproach, place, 1));
        LinearInterpolator<Transform3D<> >::Ptr pl2pld = ownedPtr(new LinearInterpolator<Transform3D<> >(place, placeDepart, 1));
        LinearInterpolator<Transform3D<> >::Ptr pld2hm = ownedPtr(new LinearInterpolator<Transform3D<> >(placeDepart, home, 1));

        cout << hm2pla->getEnd() << endl << pla2pl->getStart() << endl;
        cout << ((hm2pla->getEnd() != pla2pl->getStart())? "different" : "same") << endl;

        cout << __LINE__ << endl;
        ParabolicBlend<Transform3D<> >::Ptr blend_hm2pk = ownedPtr(new ParabolicBlend<Transform3D<> >(hm2pka, pka2pk, 0.25));
        cout << __LINE__ << endl;
        ParabolicBlend<Transform3D<> >::Ptr blend_pk2hm = ownedPtr(new ParabolicBlend<Transform3D<> >(pk2pkd, pkd2hm, 0.25));
        cout << __LINE__ << endl;
        ParabolicBlend<Transform3D<> >::Ptr blend_pkd2pla = ownedPtr(new ParabolicBlend<Transform3D<> >(pkd2hm, hm2pla, 0.25));
        cout << __LINE__ << endl;
        ParabolicBlend<Transform3D<> >::Ptr blend_hm2pl = ownedPtr(new ParabolicBlend<Transform3D<> >(hm2pla, pla2pl, 0.25));
        cout << __LINE__ << endl;
        ParabolicBlend<Transform3D<> >::Ptr blend_pl2hm = ownedPtr(new ParabolicBlend<Transform3D<> >(pl2pld, pld2hm, 0.25));
        cout << __LINE__ << endl;

        InterpolatorTrajectory<Transform3D<> > trajectory;

        if (withBlend){
            trajectory.add(hm2pka);
            trajectory.add(blend_hm2pk, pka2pk);
            trajectory.add(pk2pkd);
            trajectory.add(blend_pk2hm, pkd2hm);

            trajectory.add(/*blend_pkd2pla,*/ hm2pla);

            trajectory.add(blend_hm2pl, pla2pl);
            trajectory.add(pl2pld);
            trajectory.add(blend_pl2hm, pld2hm);

        } else {
            trajectory.add(hm2pka);
            trajectory.add(pka2pk);
            trajectory.add(pk2pkd);
            trajectory.add(pkd2hm);
            trajectory.add(hm2pla);
            trajectory.add(pla2pl);
            trajectory.add(pl2pld);
            trajectory.add(pld2hm);
        }

        return trajectory;
    }

    vector<QPath> RRTInterpolate(double extend) {
        vector<Transform3D<> > interpolationTransformsPoints = getPointTrajectorySequence();
        vector<Q> jointPoints;
        cout << __LINE__ << endl;
        for(auto tf : interpolationTransformsPoints){
            vector<Q> jointConfigs = getJointConfigurations(tf);
            cout << jointConfigs.size() << endl;
            //cout << tf << endl;
            cout << jointConfigs[0] << endl;
            jointPoints.push_back(jointConfigs[0]); // TODO change to more optimal configuration rather than simply picking the first
        }
        cout << __LINE__ << endl;
        return RRTConnect(jointPoints, extend);
    }
    vector<QPath> RRTConnect(vector<Q> interpolationJointsPoints, double extend){
        vector<QPath> totalPath;
        cout << __LINE__ << endl;
        for (int i = 0; i < interpolationJointsPoints.size() - 1; i++) {
            Q from(interpolationJointsPoints[i]);
            Q to(interpolationJointsPoints[i + 1]);

            totalPath.push_back(RRTConnectQtoQ(from, to, extend));
        }
        cout << __LINE__ << endl;
        return totalPath;
    }
    QPath RRTConnectQtoQ(Q from, Q to, double extend){
        // Inspired from RoVi sample plugin
        // and https://www.robwork.dk/manual/motionplanning/

        CollisionStrategy::Ptr cdstrategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
        CollisionDetector::Ptr collisionDetector = ownedPtr (new CollisionDetector (wc, cdstrategy));
        PlannerConstraint constraint = PlannerConstraint::make (collisionDetector, robot, state);

        QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(robot), constraint.getQConstraintPtr());
        QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

        QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner (constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

        ProximityData pdata;
        robot->setQ (from, state);
        if (collisionDetector->inCollision (state, pdata))
            RW_THROW ("Initial configuration in collision! can not plan a path.");
        robot->setQ (to, state);
        if (collisionDetector->inCollision (state, pdata))
            RW_THROW ("Final configuration in collision! can not plan a path.");

        QPath result;
        if (planner->query (from, to, result)) {
            std::cout << "Planned path with " << result.size ();
            std::cout << " configurations" << std::endl;
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
    void writeQPathToFile(vector<QPath> fullpath, string filePath){
        TimedStatePath tStatePath;
        double time = 0;
        for(int i = 0; i < fullpath.size(); i++){
            for(int j = 0; j < fullpath[i].size(); j++){
                robot->setQ(fullpath[i][j], state);
                tStatePath.push_back(TimedState(time, state));
                time += 0.01;
            }
        }
        rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, filePath);
    }

    ~MotionPlanner(){};
protected:
    void generatePoints(){
        home = robot->frames().back()->wTf(state);
        pickApproach = Transform3D<>(Vector3D<>(pick.P()(0), pick.P()(1), home.P()(2)),
                                     pick.R());

        pickDepart = Transform3D<>(pickApproach.P() /*+ Vector3D<>(0, 0, pick.P()(2))*/,
                                   pick.R());

        placeApproach = Transform3D<>(Vector3D<>(place.P()(0), place.P()(1), home.P()(2)),
                                     place.R());

        placeDepart = Transform3D<>(placeApproach.P(),
                                    home.R());
    }
    vector<Transform3D<> > getPointTrajectorySequence(){
        return {home, pickApproach, pick, pickDepart, home, placeApproach, place, placeDepart, home};
    }

    WorkCell::Ptr wc;
    SerialDevice::Ptr robot;
    State state;
    Transform3D<> home, pick, pickApproach, pickDepart, place, placeApproach, placeDepart;
};


int main(int argc, char**argv){
    // how to call the script
    if(argc < 3) {
        cout << "Usage: " << argv[0] << " <scene> <path/to/save/trajectory.dat>" << endl;
        return 0;
    }

    string scene = string(argv[1]);
    string trajectoryFilePath = string(argv[2]);

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
    //InterpolatorTrajectory<Transform3D<> > trajectory = mp.linearlyInterpolate();
    //mp.writeTrajectoryToFile(trajectory, trajectoryFilePath);
    vector<QPath> path = mp.RRTInterpolate(0.01);
    mp.writeQPathToFile(path, "qconfig.rwplay");
}
