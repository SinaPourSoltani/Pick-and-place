#include "../Robotics/MotionPlanning/motionplanning.cpp"
#if D2D
    #include "../Vision/SparseStereo/sparseStereo.cpp"
#endif
#if D3D
    #include "../Vision/DepthSensor/depthSensor.cpp"
#endif

using namespace std;

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
    
    #if D2D
        SparseStereo ss(scene);
        ss.addNoiseToImages(0,0.1);
        vector<Mat> objects = ss.stereopsis();
        for(unsigned int i = 0; i < objects.size(); i++){
            cout << objects[i] << endl;
        }
    #endif
    #if D3D
        DepthSensor ds(scene);
        ds.findObjects();
        ds.visualizePointClouds();
    #endif
    
    State state = wc->getDefaultState();
    Transform3D<> pick(Vector3D<>(0.25, 0.474, 0.191), RPY<>(0,Deg2Rad * 180,0));
    Transform3D<> place(Vector3D<>(PLACE_CENTER_X, PLACE_CENTER_Y, 0.191), RPY<>(0,Deg2Rad * 180,0));
    
    MotionPlanner mp(wc, robot, pick, place, state);
    InterpolatorTrajectory<Transform3D<> > traj = mp.linearlyInterpolate();
    mp.writeTrajectoryToFile(traj, trajectoryFilePath);
}
