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

class MotionPlanning {
public:
    MotionPlanning(){};
    MotionPlanning(SerialDevice::Ptr robot, Transform3D<> pick, Transform3D<> place, State state) :
        robot(robot),
        pick(pick),
        place(place),
        state(state) {
            generatePoints();
        };
    
    InterpolatorTrajectory<Transform3D<> > linearlyInterpolate(bool withBlend = false){
        LinearInterpolator<Transform3D<> >::Ptr hm2pka = ownedPtr(new LinearInterpolator<Transform3D<> >(home, pickApproach, 1));
        LinearInterpolator<Transform3D<> >::Ptr pka2pk = ownedPtr(new LinearInterpolator<Transform3D<> >(pickApproach, pick, 1));
        LinearInterpolator<Transform3D<> >::Ptr pk2pkd = ownedPtr(new LinearInterpolator<Transform3D<> >(pick, pickDepart, 1));
        LinearInterpolator<Transform3D<> >::Ptr pkd2hm = ownedPtr(new LinearInterpolator<Transform3D<> >(pickDepart, home, 1));
        
        LinearInterpolator<Transform3D<> >::Ptr hm2pla = ownedPtr(new LinearInterpolator<Transform3D<> >(home, placeApproach, 1));
        LinearInterpolator<Transform3D<> >::Ptr pla2pl = ownedPtr(new LinearInterpolator<Transform3D<> >(placeApproach, place, 1));
        LinearInterpolator<Transform3D<> >::Ptr pl2pld = ownedPtr(new LinearInterpolator<Transform3D<> >(place, placeDepart, 1));
        LinearInterpolator<Transform3D<> >::Ptr pld2hm = ownedPtr(new LinearInterpolator<Transform3D<> >(placeDepart, home, 1));
        
        ParabolicBlend<Transform3D<> >::Ptr blend_hm2pk = ownedPtr(new ParabolicBlend<Transform3D<> >(hm2pka, pka2pk, 0.25));
        ParabolicBlend<Transform3D<> >::Ptr blend_pk2hm = ownedPtr(new ParabolicBlend<Transform3D<> >(pk2pkd, pkd2hm, 0.25));
        
        //ParabolicBlend<Transform3D<> >::Ptr blend_pkd2pla = ownedPtr(new ParabolicBlend<Transform3D<> >(pkd2hm, hm2pla, 0.25));
        
        ParabolicBlend<Transform3D<> >::Ptr blend_hm2pl = ownedPtr(new ParabolicBlend<Transform3D<> >(hm2pla, pla2pl, 0.25));
        ParabolicBlend<Transform3D<> >::Ptr blend_pl2hm = ownedPtr(new ParabolicBlend<Transform3D<> >(pl2pld, pld2hm, 0.25));
        
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
    void writeToFile(InterpolatorTrajectory<Transform3D<> > trajectory, string filePath){
        std::ofstream out(filePath);
        for (double t = 0; t <= trajectory.duration(); t += DT) {
             Transform3D<> x = trajectory.x(t);
             out << t << " " << x.P()(0) << " " << x.P()(1) << " " << x.P()(2) << endl;
        }
        out.close();
    }
    
    ~MotionPlanning(){};
protected:
    void generatePoints(){
        home = robot->frames().back()->wTf(state);
        pickApproach = Transform3D<>(pick.P() + Vector3D<>(0, 0, home.P()(3)),
                                     pick.R());
        
        pickDepart = Transform3D<>(pickApproach.P() + Vector3D<>(0, 0, pick.P()(3)),
                                   pick.R());
        
        placeApproach = Transform3D<>(place.P() + Vector3D<>(0, 0, home.P()(3)),
                                     place.R());
        
        placeDepart = Transform3D<>(placeApproach.P(),
                                    home.R());
    }
    
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
    State state = wc->getDefaultState();
    
    Transform3D<> pick(Vector3D<>(0.25, 0.474, 0.191), RPY<>(0,0,0));
    Transform3D<> place(Vector3D<>(PLACE_CENTER_X, PLACE_CENTER_Y, 0.191), RPY<>(0,0,0));
    
    MotionPlanning mp(robot, pick, place, state);
    InterpolatorTrajectory<Transform3D<> > trajectory = mp.linearlyInterpolate();
    mp.writeToFile(trajectory, trajectoryFilePath);
}
