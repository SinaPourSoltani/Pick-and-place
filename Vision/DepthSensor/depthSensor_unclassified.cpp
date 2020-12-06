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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <thread>
#include <random>
#include <limits>

#define Z_MIN -2
#define Z_TABLE -1.37
#define Y_OBSTACLE -0.18

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
ModelCoefficients::Ptr planeModelSegmentation(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr plane){
    PointCloud<PointXYZ>::Ptr planeModel (new PointCloud<PointXYZ>);
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    // Create the segmentation object
    SACSegmentation<PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        return coefficients;
    }
    
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    << coefficients->values[1] << " "
    << coefficients->values[2] << " "
    << coefficients->values[3] << std::endl;
    
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (const auto& idx: inliers->indices) {
        std::cerr << idx << "    " << cloud->points[idx].x << " "
        << cloud->points[idx].y << " "
        << cloud->points[idx].z << std::endl;
        planeModel->push_back(cloud->points[idx]);
    }
    
    *plane = *planeModel;
    
    return coefficients;
}

float sq(float x){
    return x*x;
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
PointXYZ cross(PointXYZ a, PointXYZ b){
    return PointXYZ(a.y*b.z - a.z*b.y, a.x*b.z - a.z*b.x, a.x*b.y - a.y*b.x);
}
float dot(PointXYZ a, PointXYZ b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
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
void removePoints(PointCloud<PointXYZ>::Ptr cloud, ModelCoefficients::Ptr plane, PointXYZ planePoint, bool behind = true){
    PointCloud<PointXYZ>::Ptr reducedCloud(new PointCloud<PointXYZ>);
    double a = plane->values[0];
    double b = plane->values[1];
    double c = plane->values[2];
    //double d = plane->values[3];
    
    int i = 0;
    for(auto point : cloud->points) {
        PointXYZ planePoint2point = sub(point, planePoint);
        double dot = a * planePoint2point.x + b * planePoint2point.y + c * planePoint2point.z;
        cout << i++ << " Dir: " << dot << endl;
        if(behind){
            if( dot > 0 ){
                reducedCloud->push_back(point);
            }
        } else {
            if(dot < -0.1 ){
                reducedCloud->push_back(point);
            }
        }
    }
    *cloud = *reducedCloud;
}
vector<PointCloud<PointXYZ>::Ptr> cluster(PointCloud<PointXYZ>::Ptr cloud){
    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    vector<PointCloud<PointXYZ>::Ptr> objectClouds = {};
    for(int i = 0; i < cluster_indices.size(); i++){
        PointCloud<PointXYZ>::Ptr object(new PointCloud<PointXYZ>);
        for(int j = 0; j < cluster_indices[i].indices.size(); j++){
            object->push_back(cloud->points[cluster_indices[i].indices[j]]);
        }
        objectClouds.push_back(object);
    }
    
    return objectClouds;
}
PointXYZ centroid(PointCloud<PointXYZ>::Ptr cloud){
    PointXYZ centroid(0,0,0);
    int num_points = cloud->size();
    for(int i = 0; i < num_points; i++){
        centroid = add(centroid, cloud->points[i]);
    }
    centroid = divide(centroid, num_points);
    return centroid;
}
vector<PointCloud<PointXYZ>::Ptr> sortClusters(vector<PointCloud<PointXYZ>::Ptr> objectClouds, int axis, bool increasingOrder=true){
    vector<float> sortingAxisCentroid = {};
    vector<int> objectIdx = {};
    
    for(int i = 0; i < objectClouds.size(); i++){
        PointXYZ c = centroid(objectClouds[i]);
        Vector3D<float> centroid(c.x, c.y, c.z);
        sortingAxisCentroid.push_back(centroid[axis]);
        objectIdx.push_back(i);
    }
    
    
    // sort
    int N = sortingAxisCentroid.size();
    bool swapped = true;
    while(swapped){
        swapped = false;
        for(int i = 1; i < N; i++){
            if( ( increasingOrder && sortingAxisCentroid[i-1] > sortingAxisCentroid[i] )
               || ( !increasingOrder && sortingAxisCentroid[i-1] < sortingAxisCentroid[i] )){
                // swap object axis value
                float tmp = sortingAxisCentroid[i];
                sortingAxisCentroid[i] = sortingAxisCentroid[i-1];
                sortingAxisCentroid[i-1] = tmp;
                
                // swap corresponding object index
                int tmpIdx = objectIdx[i];
                objectIdx[i] = objectIdx[i-1];
                objectIdx[i-1] = tmpIdx;
                
                swapped = true;
            }
        }
    }
    
    cout << "after sorting" << endl;
    for (auto centroid : sortingAxisCentroid) {
        cout << centroid << endl;
    }
    
    
    vector<PointCloud<PointXYZ>::Ptr> sortedClouds = {};
    // put clouds into new sorted vector
    for (int i = 0; i < N; i++) {
        sortedClouds.push_back(objectClouds[objectIdx[i]]);
    }
    return sortedClouds;
}
PointXYZ maxPointInAxis(PointCloud<PointXYZ>::Ptr cloud){
    PointXYZ maxPointInAxis(0,0,0);
    for(auto point : cloud->points) {
        if (abs(point.z) > maxPointInAxis.z) {
            maxPointInAxis = point;
        }
    }
    return maxPointInAxis;
}
void removePointsInAxis(PointCloud<PointXYZ>::Ptr cloud, PointXYZ maxPoint, bool behind=true){
    PointCloud<PointXYZ>::Ptr reducedCloud(new PointCloud<PointXYZ>);
    for(auto point : cloud->points) {
        if (point.z > maxPoint.z) {
            reducedCloud->push_back(point);
        }
    }
    *cloud = *reducedCloud;
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
    PointXYZ c = centroid(cloud);
    int num_points = cloud->size();
    float sum = 0;
    for(int i = 0; i < num_points; i++){
        PointXYZ point = cloud->points[i];
        sum += sq(point.x - c.x) + sq(point.y - c.y) + sq(point.z - c.z);
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
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    vector<PointCloud<PointXYZ>::Ptr> objectClouds;
    
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
        //voxelGrid(cloud);
        //passthrough(cloud);
        //addNoise(cloud, 0.01);
        //plane = detectPlane(cloud, pclNormals, randomPoints, 50000, 0.0001);
        
        coefficients = planeModelSegmentation(cloud, plane);
        //PointXYZ mostZPoint = maxPointInAxis(plane);
        removePoints(cloud, coefficients, plane->points[0], true);
        
        
        coefficients = planeModelSegmentation(cloud, plane);
        removePoints(cloud, coefficients, plane->points[0], false);
        objectClouds = cluster(cloud);
        cout << "BEFORE: " << endl;
        for (auto object : objectClouds) {
            cout << centroid(object) << endl;
        }
        objectClouds = sortClusters(objectClouds, 0, false);
        
        cout << "AFTER: " << endl;
        for (auto object : objectClouds) {
            randomPoints->push_back(centroid(object));
            cout << centroid(object) << endl;
        }
        
        cout << "K: " << objectClouds.size() << endl;
        
        //removePointsInAxis(cloud, mostZPoint, true);
        //pclNormals = calculateSurfaceNormals(cloud);
        app.close();
    });
    t.detach();
    app.run();
    
    PCLVisualizer v("PointCloud");
    //v.addPointCloud<PointXYZ>(cloud, "points");
    
    vector<string> colors = {"Red","Green","Blue"};
    for(int i = 0; i < objectClouds.size(); i++){
        v.addPointCloud<PointXYZ>(objectClouds[i],
                                  PointCloudColorHandlerCustom<PointXYZ>(objectClouds[i],
                                                                         ((i == 0) ? 255 : 0),
                                                                         ((i == 1) ? 255 : 0),
                                                                         ((i == 2) ? 255 : 0))
                                  , "object" + colors[i]);
    }
    
    //v.addPointCloud<PointXYZ>(plane, PointCloudColorHandlerCustom<PointXYZ>(plane, 0,255, 0),"plane");
    //v.addPlane(*coefficients, "detectedPlane");
    v.addPointCloud<PointXYZ>(randomPoints, PointCloudColorHandlerCustom<PointXYZ>(randomPoints, 255,255, 255),"randompoints");
    v.addPointCloudNormals<PointNormal>(pclNormals,1,0.6,"normals");
    v.spin();
     
    return 0;
}
