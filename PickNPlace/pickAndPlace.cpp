#include "../Robotics/MotionPlanning/motionplanning.cpp"
#if D2D
    #include "../Vision/SparseStereo/sparseStereo.cpp"
#endif
#if D3D
    #include "../Vision/DepthSensor/depthSensor.cpp"
#endif

#include "fstream"

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

double euclideanDistance(Transform3D<> t1, Transform3D<> t2){
    double xDist = (t1.P()(0) - t2.P()(0));
    double yDist = (t1.P()(1) - t2.P()(1));
    double zDist = (t1.P()(2) - t2.P()(2));
    
    double dist = sqrt( xDist * xDist + yDist * yDist + zDist * zDist );
    return dist;
}

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
    
    ofstream file;
    file.open("DepthSensorTestBIG1.txt");
    file << "noise, trial, errorR, errorG, errorB" << endl;

    #if D2D
        SparseStereo ss(scene);
        ss.addNoiseToImages(0,0.0);
        vector<Mat> objects = ss.stereopsis();
        for(unsigned int i = 0; i < objects.size(); i++){
            cout << objects[i] << endl;
        }
        picks = convertMatToTransform(objects);
        // set pick points
    #endif
    #if D3D
        string folderToModels = "../../Vision/DepthSensor/";
        DepthSensor ds(scene);
        bool once = true;
        for (float noise = 0.007; noise <= 0.02; noise += 0.001) {
            for (unsigned int trial = 0; trial < 20; trial++) {
                if (noise == 0.002 && once){
                    trial = 17; // husk at ændre filnavn!!!!
                    once = false;
                }
                file << noise << "\t" << trial << "\t";
                vector<Transform3D<> > objectTransforms = {};
                objectTransforms = ds.findObjects(noise, folderToModels);
                //picks = objectTransforms;
                for (unsigned int i = 0; i < objectTransforms.size(); i++ ) { // not necessary if relative to Table
                    Transform3D<> zCorrected(Vector3D<>(objectTransforms[i].P()(0),objectTransforms[i].P()(1),objectTransforms[i].P()(2) + 0.1), objectTransforms[i].R());
                    objectTransforms[i] = zCorrected;
                }
                for (unsigned int i = 0; i < objectTransforms.size(); i++ ){
                    double error = euclideanDistance(objectTransforms[i], picks[i]);
                    cout << i << ": " << error << endl;
                    file << error << "\t";
                }
                file << endl;
            }
        }
        // set pick points
    #endif
    
    file.close();

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
