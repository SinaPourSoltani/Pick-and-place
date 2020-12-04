//
//  depthSensor.cpp
//  Robotics and Computer Vision
//
//  Created by Sina Pour Soltani on 13/11/2020.
//  1st semester Master's programme in Advanced Robotics Technology
//

#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/core/PropertyMap.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <thread>
#include <random>
#include <limits>

#define Z_MIN -2
#define Z_TABLE -1.37
#define Y_OBSTACLE -0.18

using namespace std;

using namespace pcl;
//using namespace pcl::common;
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

void grapPointCloud(PointCloud<PointXYZ>::Ptr cloud, WorkCell::Ptr wc, RobWorkStudio* rwstudio, Frame* cameraFrame, double fovy, int width, int height){
    // Inspiration from: https://www.robwork.dk/manual/simulated_sensors/
    PointCloud<PointXYZ>::Ptr pclScene(new PointCloud<PointXYZ>);
    
    State state = wc->getDefaultState();
    SceneViewer::Ptr gldrawer = rwstudio->getView()->getSceneViewer();
    GLFrameGrabber25D::Ptr framegrabber25D = ownedPtr( new GLFrameGrabber25D(width,height,fovy) );
    framegrabber25D->init(gldrawer);
    
    framegrabber25D->grab(cameraFrame, state);
    rw::geometry::PointCloud* img = &(framegrabber25D->getImage());
    for(auto p : img->getData()) {
        if (p(2) > Z_MIN) {
            PointXYZ pcl_p;
            pcl_p.x = p(0);
            pcl_p.y = p(1);
            pcl_p.z = p(2);
            pclScene->push_back(pcl_p);
        }
    }
    *cloud = *pclScene;
}

PointCloud<PointNormal>::Ptr calculateSurfaceNormals(PointCloud<PointXYZ>::Ptr cloud){
    // Inspiration from: https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
    // Create the normal estimation class, and pass the input dataset to it
    NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (cloud);
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Output datasets
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    
    // Compute the features
    ne.compute (*cloud_normals);
    
    PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
    concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
    
    return cloud_with_normals;
}

void voxelGrid(PointCloud<PointXYZ>::Ptr cloud){
    // Inspiration from: https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html?
    cout << "Creating voxel grid.." << endl;
    cout << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
    // Create the filtering object
    VoxelGrid<PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);
    
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
    
    *cloud = *cloud_filtered;
}
void addNoise(PointCloud<PointXYZ>::Ptr cloud, double std){
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> distribution(0,std);
    
    for(int i = 0; i < cloud->size(); i++){
        double noisex = distribution(gen);
        double noisey = distribution(gen);
        double noisez = distribution(gen);
        
        cloud->points[i].x += noisex;
        cloud->points[i].y += noisey;
        cloud->points[i].z += noisez;
    }
}
void passthrough(PointCloud<PointXYZ>::Ptr cloud){
    PointCloud<PointXYZ>::Ptr objectsPassthrough(new PointCloud<PointXYZ>);
    for(auto &p : cloud->points) {
        if (p.y > Y_OBSTACLE && p.z > Z_TABLE){
            cout << p << endl;
            objectsPassthrough->push_back(p);
        }
    }
    *cloud = *objectsPassthrough;
}

PointXYZ add(PointXYZ a, PointXYZ b){
    return PointXYZ(a.x + b.x, a.y + b.y, a.z + b.z);
}
PointXYZ sub(PointXYZ a, PointXYZ b){
    return PointXYZ(a.x - b.x, a.y - b.y, a.z - b.z);
}
PointXYZ divide(PointXYZ a, int d){
    return PointXYZ(a.x / d, a.y / d, a.z / d);
}
float sq(float x){
    return x*x;
}
PointXYZ cross(PointXYZ a, PointXYZ b){
    return PointXYZ(a.y*b.z - a.z*b.y, a.x*b.z - a.z*b.x, a.x*b.y - a.y*b.x);
}
float dot(PointXYZ a, PointXYZ b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

PointNormal points2plane(vector<PointXYZ> threepoints){
    PointXYZ a = threepoints[0];
    PointXYZ b = threepoints[1];
    PointXYZ c = threepoints[2];
    
    PointXYZ u = sub(b, a);
    PointXYZ v = sub(c, a);
    
    PointXYZ n = cross(u, v);
    
    return PointNormal(a.x, a.y, a.z, n.x, n.y, n.z);
}
float dist2plane(PointNormal* plane, PointXYZ* point){
    cout << "plane: " << *plane << endl << "and point: " << *point << endl;
    PointXYZ normal(plane->normal[0], plane->normal[1], plane->normal[2]);
    PointXYZ unitPoint(point->x - plane->x, point->y - plane->y, point->z - plane->z);
    return dot(normal, unitPoint);
}
PointCloud<PointXYZ>::Ptr find_inliers(PointNormal plane, PointCloud<PointXYZ>::Ptr cloud, float threshold){
    PointCloud<PointXYZ>::Ptr inliers(new PointCloud<PointXYZ>);
    for(auto point : cloud->points){
        float dist = dist2plane(&plane, &point);
        cout << "distance: " << dist << endl;
        if (abs(dist) <= threshold){
            inliers->push_back(point);
        }
    }
    return inliers;
}

float calc_std(PointCloud<PointXYZ>::Ptr cloud){
    // Tested and works
    PointXYZ centroid(0,0,0);
    int num_points = cloud->size();
    for(int i = 0; i < num_points; i++){
        centroid = add(centroid, cloud->points[i]);
    }
    centroid = divide(centroid, num_points);
    float sum = 0;
    for(int i = 0; i < num_points; i++){
        PointXYZ point = cloud->points[i];
        sum += sq(point.x - centroid.x) + sq(point.y - centroid.y) + sq(point.z - centroid.z);
    }
    sum /= num_points;
    
    return sqrt(sum);
}

PointCloud<PointXYZ>::Ptr detectPlane(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointNormal>::Ptr bplane,PointCloud<PointXYZ>::Ptr brp, int forseeable_support, float threshold, float alpha = 0.95) {
    float bestSupport = 0;
    PointNormal bestPlane(0,0,0);
    float bestStd = numeric_limits<float>::infinity();
    float eps = 1 - (float)forseeable_support / cloud->size();
    int N = round(log(1 - alpha) / log(1 - pow((1 - eps),3)));
    
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distr(0, cloud->size() - 1);
    
    PointCloud<PointXYZ>::Ptr inliers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr mostInliers(new PointCloud<PointXYZ>);
    vector<PointXYZ> bestRandomPoints = {};
    
    for(int i = 0; i < 1; i++){
        vector<PointXYZ> randomPoints = {};
        for(int j = 0; j < 3; j++) { randomPoints.push_back(cloud->points[distr(gen)]);}
        PointNormal plane = points2plane(randomPoints);
        inliers = find_inliers(plane, cloud, threshold);
        float sd = calc_std(inliers);
        int num_inliers = inliers->size();
        if(num_inliers > bestSupport || (num_inliers == bestSupport && sd < bestStd)){
            cout << "Found plane with #" << num_inliers << " inliers." << endl;
            bestSupport = num_inliers,
            bestStd = sd;
            bestPlane = plane;
            bestRandomPoints = {};
            for(int j = 0; j < 3; j++) { bestRandomPoints.push_back(randomPoints[j]);}
            *mostInliers = *inliers;
        }
    }
    cout << "BP" << endl;
    cout << bestPlane << endl;
    bplane->push_back(bestPlane);
    cout << "BRP" << endl;
    for(int j = 0; j < 3; j++) {
        cout << j+1 << " " << bestRandomPoints[j] << endl;
        brp->push_back(bestRandomPoints[j]);
    }
    return mostInliers;
}

int main(int argc, char**argv) {
    if(argc < 2){
        cout << "Usage: " << argv[0] << " <scene>" << endl;
        return 0;
    }
    string WC_FILE = argv[1];
    string camera25D = "Scanner25D";
    
    // Load workcell
    const WorkCell::Ptr wc = WorkCellLoader::Factory::load(WC_FILE);
    if (wc.isNull())
        RW_THROW("WorkCell could not be loaded.");
    
    // Load 25D camera frame
    Frame* const cameraFrame = wc->findFrame(camera25D);
    if (cameraFrame == nullptr)
        RW_THROW("Camera frame could not be found.");
    
    // Read camera properties from <scene>
    const PropertyMap& properties = cameraFrame->getPropertyMap();
    if (!properties.has(camera25D))
        RW_THROW("Camera frame does not have Camera property.");
    const string parameters = properties.get<string>(camera25D);
    
    istringstream iss (parameters, istringstream::in);
    double fovy; int width, height;
    iss >> fovy >> width >> height;
    cout << "Camera properties: fov " << fovy << " width " << width << " height " << height << endl;
    
    // Initialized globally to allow for main to visualize when thread overwrites with simulated point cloud
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr plane(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr randomPoints(new PointCloud<PointXYZ>);
    PointCloud<PointNormal>::Ptr pclNormals(new PointCloud<PointNormal>);
    
    RobWorkStudioApp app("");
    thread t([&]() {
        // Wait for app to launch
        while (!app.isRunning()) {
            TimerUtil::sleepMs(1000);
            cout << "Waiting for launch.." << endl;
        }
        RobWorkStudio* rwstudio = app.getRobWorkStudio();
        rwstudio->postOpenWorkCell(WC_FILE);
        TimerUtil::sleepMs(2000);
        
        grapPointCloud(cloud, wc, rwstudio, cameraFrame, fovy, width, height);
        voxelGrid(cloud);
        //passthrough(cloud);
        //addNoise(cloud, 0.01);
        plane = detectPlane(cloud, pclNormals, randomPoints, 50000, 0.0001);
        
        //pclNormals = calculateSurfaceNormals(cloud);
        app.close();
    });
    t.detach();
    app.run();
    
    PCLVisualizer v("PointCloud");
    v.addPointCloud<PointXYZ>(cloud, "points");
    v.addPointCloud<PointXYZ>(plane, PointCloudColorHandlerCustom<PointXYZ>(plane, 0,255, 0),"plane");
    v.addPointCloud<PointXYZ>(randomPoints, PointCloudColorHandlerCustom<PointXYZ>(plane, 255,0, 0),"randompoints");
    v.addPointCloudNormals<PointNormal>(pclNormals,1,0.6,"normals");
    v.spin();

    return 0;
}
