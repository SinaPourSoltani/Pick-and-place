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

#define P2P "P2P"
#define P2PB "P2PB"
#define RRT "RRT"

#define CYLR_HEIGHT 0.16
#define CYLG_HEIGHT 0.08
#define CYLB_HEIGHT 0.12
#define CYL_OFFSET 0.0001

int main(int argc, char** argv){
    if(argc < 2) {
        cout << "Usage: " << argv[0] << " <scene> [motion planning method]" << endl;
        return 0;
    }

    string scene = string(argv[1]);
    string mpm = P2P;

    if(argc > 2){
        string input = string(argv[2]);
        if(input == P2P || input == P2PB || input == RRT){
            mpm = input;
        } else {
            throw("Undefined Motion Planning Methods [default: P2P] :\n Choose between:\n - P2P\n - P2PB\n - RRT\n");
        }
    }

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

    // Predefined place locations

    Transform3D<> placeRed(Vector3D<>(PLACE_CENTER_X,
                                      PLACE_CENTER_Y,
                                      0.191),
                           RPY<>(0,Deg2Rad * 180,0));
    Transform3D<> placeGreen(Vector3D<>(PLACE_CENTER_X,
                                        PLACE_CENTER_Y,
                                        placeRed.P()(2) + CYLR_HEIGHT / 2 + CYLG_HEIGHT / 2 + CYL_OFFSET),
                             RPY<>(0,Deg2Rad * 180,0));
    Transform3D<> placeBlue(Vector3D<>(PLACE_CENTER_X,
                                       PLACE_CENTER_Y,
                                       placeGreen.P()(2) + CYLG_HEIGHT / 2 + CYLB_HEIGHT / 2 + CYL_OFFSET),
                            RPY<>(0,Deg2Rad * 180,0));
    vector<Transform3D<> > places = {placeRed, placeGreen, placeBlue};

    // Vision determined pick locations
    vector<string> objectNames = {"CylinderRed", "CylinderGreen", "CylinderBlue"};
    vector<Transform3D<> > picks = {
        Transform3D<>(Vector3D<>(0.25, 0.474, 0.191), RPY<>(0,Deg2Rad * 180,0)),
        Transform3D<>(Vector3D<>(0, 0.474, 0.150), RPY<>(0,Deg2Rad * 180,0)),
        Transform3D<>(Vector3D<>(-0.25, 0.474, 0.170), RPY<>(0,Deg2Rad * 180,0)),
    };

    #if D2D
        SparseStereo ss(scene);
        ss.addNoiseToImages(0,0.1);
        vector<Mat> objects = ss.stereopsis();
        for(unsigned int i = 0; i < objects.size(); i++){
            cout << objects[i] << endl;
        }
        // set pick points
    #endif
    #if D3D
        DepthSensor ds(scene);
        ds.findObjects();
        ds.visualizePointClouds();
        // set pick points
    #endif

    State state = wc->getDefaultState();
    MotionPlanner mp(wc, robot, state);
    vector<float> timeBetweenPoints = {1, 1, 1, 1, 1, 1, 1, 1};
    vector<float> blendFractions = {0.0, 0.5, 0.0, 0.5, 0.5, 0.5, 0.0, 0.5};

    if(mpm == RRT){
        TimedStatePath tStatePaths;
        for(unsigned int i = 0; i < picks.size(); i++){
            mp.setPickAndPlace(picks[i], objectNames[i], places[i]);
            vector<QPath> qpaths = mp.RRTInterpolate(0.05);
            TimedStatePath tStatePath = mp.writeQPathVectorToFile(qpaths);
            for(unsigned int p = 0; p < tStatePath.size(); p++){
              tStatePaths.push_back(tStatePath[p]);
            }
        }
        mp.writeTimedStatePath(tStatePaths, qpathFilePath);

    } else if(mpm == P2PB) {
        for(unsigned int i = 0; i < picks.size(); i++){
            mp.setPickAndPlace(picks[i], objectNames[i], places[i]);
            InterpolatorTrajectory<Transform3D<> > traj = mp.linearlyInterpolate(timeBetweenPoints, blendFractions);
            QPath path = mp.trajectoryToQPath(traj);
            mp.writeTrajectoryToFile(traj, trajectoryFilePath);
            mp.writeQPathToFile(path, qpathFilePath);
        }
    } else {
        TimedStatePath tStatePaths;
        InterpolatorTrajectory<Transform3D<> > trajectories;
        for(unsigned int i = 0; i < picks.size(); i++){
            mp.setPickAndPlace(picks[i], objectNames[i], places[i]);
            InterpolatorTrajectory<Transform3D<> > traj = mp.linearlyInterpolate(timeBetweenPoints);
            trajectories.add(&traj);
            QPath path = mp.trajectoryToQPath(traj);
            TimedStatePath tStatePath = mp.writeQPathToFile(path, qpathFilePath);
            for(unsigned int p = 0; p < tStatePath.size(); p++){
              tStatePaths.push_back(tStatePath[p]);
            }
        }
        mp.writeTrajectoryToFile(trajectories, trajectoryFilePath);
        mp.writeTimedStatePath(tStatePaths, qpathFilePath);
    }
}
