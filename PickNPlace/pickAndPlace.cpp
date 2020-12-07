#include "../Robotics/MotionPlanning/motionplanning.cpp"
#include "../Vision/DepthSensor/depthSensor.cpp"
//#include "../Vision/SparseStereo/sparseStereo.cpp"

using namespace std;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;

using namespace rw::core;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rws;

using rw::graphics::SceneViewer;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;

int main(int argc, char** argv){
    cout << "Hello world" << endl;
    
    if(argc < 2) {
        cout << "Usage: " << argv[0] << " <scene>" << endl;
        return 0;
    }

    string scene = string(argv[1]);
    string trajectoryFilePath = "traj.dat";
    string qpathFilePath = "qconfig.rwplay";

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

    Math::seed();
    
    DepthSensor ds(scene);
    ds.findObjects();
    ds.visualizePointClouds();
    
    
    State state = wc->getDefaultState();
    Transform3D<> pick(Vector3D<>(0.25, 0.474, 0.191), RPY<>(0,Deg2Rad * 180,0));
    Transform3D<> place(Vector3D<>(PLACE_CENTER_X, PLACE_CENTER_Y, 0.191), RPY<>(0,Deg2Rad * 180,0));
    
    MotionPlanner mp(wc, robot, pick, place, state);
    InterpolatorTrajectory<Transform3D<> > traj = mp.linearlyInterpolate();
    mp.writeTrajectoryToFile(traj, trajectoryFilePath);
}
