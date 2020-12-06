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

class DepthSensor {
public:
    DepthSensor(){};
    DepthSensor(string workcellFilePath) :
    workcellFilePath(workcellFilePath){
        loadWorkcellAndCamera();
        mallocPointClouds();
        
        RobWorkStudioApp app("");
        thread t([&](){
            // Wait for app to launch
            while (!app.isRunning()) {
                TimerUtil::sleepMs(1000);
                cout << "Waiting for launch.." << endl;
            }
            RobWorkStudio* rwstudio = app.getRobWorkStudio();
            rwstudio->postOpenWorkCell(workcellFilePath);
            TimerUtil::sleepMs(2000);
            
            grabPointCloud(rwstudio);
            
            app.close();
        });
        t.detach();
        app.run();
    };
    
    void findObjects(){
        //addNoise(0.001); // 0.001 -> 0.02 | 1 mm to 2 cm
        //voxelGrid();
        //calculateSurfaceNormals();
        
        
        // Find biggest planar surface (Tabletop)
        // And remove points beneath
        planeModelSegmentation();
        removePoints(true);
        
        // Find biggest remaining planar surface (Obstacle)
        // And remove points behind
        planeModelSegmentation();
        removePoints(false);
        
        cluster();
        sortClusters(0, false);
        
        // Print object centroids
        vector<PointXYZ> centroids = getCentroidsOfObjects();
        for (auto c : centroids) {
            cout << c << endl;
        }
        
    }
    void visualizePointClouds(){
        PCLVisualizer v("PointCloud");
        v.addPointCloud<PointXYZ>(cloud, "points");
        vector<string> colors = {"Red","Green","Blue"};
        for(int i = 0; i < objectClouds.size(); i++){
            v.addPointCloud<PointXYZ>(objectClouds[i], PointCloudColorHandlerCustom<PointXYZ>(objectClouds[i],
                                                                                              ((i == 0) ? 255 : 0),
                                                                                              ((i == 1) ? 255 : 0),
                                                                                              ((i == 2) ? 255 : 0))
                                      , "object_" + colors[i]);
        }
        //v.addPointCloud<PointXYZ>(plane, PointCloudColorHandlerCustom<PointXYZ>(plane, 0,255, 0),"plane");
        //v.addPlane(*coefficients, "detectedPlane");
        //v.addPointCloud<PointXYZ>(randomPoints, PointCloudColorHandlerCustom<PointXYZ>(randomPoints, 255,255, 255),"randompoints");
        v.addPointCloudNormals<PointNormal>(pclNormals,1,0.6,"normals");
        v.spin();
    }
    
    ~DepthSensor(){};

private:
    // Helper Functions
    void loadWorkcellAndCamera(){
        // Load workcell
        wc = WorkCellLoader::Factory::load(workcellFilePath);
        if (wc.isNull())
            RW_THROW("WorkCell could not be loaded.");
        
        // Load 25D camera frame
        cameraFrame = wc->findFrame(camera25D);
        if (cameraFrame == nullptr)
            RW_THROW("Camera frame could not be found.");
        
        // Read camera properties from <scene>
        const PropertyMap& properties = cameraFrame->getPropertyMap();
        if (!properties.has(camera25D))
            RW_THROW("Camera frame does not have Camera property.");
        const string parameters = properties.get<string>(camera25D);
        
        istringstream iss (parameters, istringstream::in);
        iss >> fovy >> width >> height;
        cout << "Camera properties: fov " << fovy << " width " << width << " height " << height << endl;
    };
    void mallocPointClouds(){
        cloud = ownedPtr(new PointCloud<PointXYZ>);
        plane = ownedPtr(new PointCloud<PointXYZ>);
        randomPoints = ownedPtr(new PointCloud<PointXYZ>);
        pclNormals = ownedPtr(new PointCloud<PointNormal>);
        coefficients = ownedPtr(new ModelCoefficients);
    }
    
    // Point Cloud Specific
    void grabPointCloud(RobWorkStudio* rwstudio){
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
    void voxelGrid(){
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
    void addNoise(double std){
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
    
    void calculateSurfaceNormals(){
        // Inspiration from: https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
        
        NormalEstimation<PointXYZ, Normal> ne;
        ne.setInputCloud (cloud);
        
        search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
        ne.setSearchMethod (tree);
        
        PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
        ne.setRadiusSearch (0.03);
        ne.compute (*cloud_normals);
        
        PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
        concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
        
        *pclNormals = *cloud_with_normals;
    }
    void planeModelSegmentation(){
        // Inspiration from: https://pcl.readthedocs.io/en/latest/planar_segmentation.html
        PointCloud<PointXYZ>::Ptr planeModel (new PointCloud<PointXYZ>);
        ModelCoefficients::Ptr coeffs (new ModelCoefficients);
        PointIndices::Ptr inliers (new PointIndices);
        SACSegmentation<PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (SACMODEL_PLANE);
        seg.setMethodType (SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coeffs);
        
        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
            return;
        }
        /*
        std::cerr << "Model coefficients: " << coeffs->values[0] << " "
        << coeffs->values[1] << " "
        << coeffs->values[2] << " "
        << coeffs->values[3] << std::endl;
        */
        //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
        for (const auto& idx: inliers->indices) {
            //std::cerr << idx << "    " << cloud->points[idx].x << " "
            //<< cloud->points[idx].y << " "
            //<< cloud->points[idx].z << std::endl;
            planeModel->push_back(cloud->points[idx]);
        }
        
        
        *plane = *planeModel;
        *coefficients = *coeffs;
    }
    void removePoints(bool behind = true){
        PointCloud<PointXYZ>::Ptr reducedCloud(new PointCloud<PointXYZ>);
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        //double d = coefficients->values[3];
        PointXYZ planePoint = plane->points[0];
        
        for(auto point : cloud->points) {
            PointXYZ planePoint2point = sub(point, planePoint);
            double dot = a * planePoint2point.x + b * planePoint2point.y + c * planePoint2point.z;
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
    
    void cluster(){
        // Inspiration from: https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html
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
        
        vector<PointCloud<PointXYZ>::Ptr> objClouds = {};
        for(int i = 0; i < cluster_indices.size(); i++){
            PointCloud<PointXYZ>::Ptr object(new PointCloud<PointXYZ>);
            for(int j = 0; j < cluster_indices[i].indices.size(); j++){
                object->push_back(cloud->points[cluster_indices[i].indices[j]]);
            }
            objClouds.push_back(object);
        }
        objectClouds = objClouds;
    }
    void sortClusters(int axis, bool increasingOrder=true){
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
        
        vector<PointCloud<PointXYZ>::Ptr> sortedClouds = {};
        // put clouds into new sorted vector
        for (int i = 0; i < N; i++) {
            sortedClouds.push_back(objectClouds[objectIdx[i]]);
        }
        objectClouds = sortedClouds;
    }
    
    vector<PointXYZ> getCentroidsOfObjects(){
        vector<PointXYZ> centroids;
        for(auto object : objectClouds){
            centroids.push_back(centroid(object));
        }
        return centroids;
    }
    
    // Mathematical Helpers
    float sq(float x){
        return x*x;
    }
    float dot(PointXYZ a, PointXYZ b){
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    PointXYZ cross(PointXYZ a, PointXYZ b){
        return PointXYZ(a.y*b.z - a.z*b.y, a.x*b.z - a.z*b.x, a.x*b.y - a.y*b.x);
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
    
    PointXYZ centroid(PointCloud<PointXYZ>::Ptr cloud){
        PointXYZ centroid(0,0,0);
        int num_points = cloud->size();
        for(int i = 0; i < num_points; i++){
            centroid = add(centroid, cloud->points[i]);
        }
        centroid = divide(centroid, num_points);
        return centroid;
    }
    
    // Attributes
    WorkCell::Ptr wc;
    Frame* cameraFrame;
    string workcellFilePath;
    string camera25D = "Scanner25D";
    double fovy; int width, height;

    PointCloud<PointXYZ>::Ptr cloud;
    PointCloud<PointXYZ>::Ptr plane;
    PointCloud<PointXYZ>::Ptr randomPoints;
    PointCloud<PointNormal>::Ptr pclNormals;
    ModelCoefficients::Ptr coefficients;
    vector<PointCloud<PointXYZ>::Ptr> objectClouds;
};

int main(int argc, char**argv) {
    if(argc < 2){
        cout << "Usage: " << argv[0] << " <scene>" << endl;
        return 0;
    }
    string WC_FILE = argv[1];
    /*
    // Load
    PointCloud<PointXYZ>::Ptr object(new PointCloud<PointXYZ>);
    loadPCDFile(argv[1], *object);
    
    // Show
    PCLVisualizer v("Before local alignment");
    v.addPointCloud<PointXYZ>(object, PointCloudColorHandlerCustom<PointXYZ>(object, 0, 255, 0), "object");
    v.spin();
    */
    DepthSensor ds(WC_FILE);
    ds.findObjects();
    ds.visualizePointClouds();
     
    return 0;
}

/*
 Ressources:
 https://www.robwork.dk/manual/simulated_sensors/
 https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html?
 https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
 https://pcl.readthedocs.io/en/latest/planar_segmentation.html
 https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html
*/
