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
#include <pcl/common/transforms.h>
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
    
    vector<Transform3D<> > findObjects(string pathToModelsFolder = ""){
        //addNoise(0.001); // 0.001 -> 0.02 | 1 mm to 2 cm
        //visualizePointClouds();
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
        
        if(pathToModelsFolder != ""){
            loadObjectModels(pathToModelsFolder);
            moveObjectModels();
            for (unsigned int i = 0; i < objectClouds.size(); i++) {
                ICP(objectClouds[i], objectModels[i], 500);
            }
        }
        
        return getObjectsAsTransforms();
    }
    void visualizePointClouds(){
        PCLVisualizer v("PointCloud");
        v.addPointCloud<PointXYZ>(cloud, "points");
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
        for(int i = 0; i < objectModels.size(); i++){
            v.addPointCloud<PointXYZ>(objectModels[i], PointCloudColorHandlerCustom<PointXYZ>(objectModels[i], 255, 255, 255), "objectModel_" + colors[i]);
        }
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
        cloud = ownedPtr(new pcl::PointCloud<PointXYZ>);
        plane = ownedPtr(new pcl::PointCloud<PointXYZ>);
        randomPoints = ownedPtr(new pcl::PointCloud<PointXYZ>);
        pclNormals = ownedPtr(new pcl::PointCloud<PointNormal>);
        coefficients = ownedPtr(new ModelCoefficients);
    }
    
    // Point Cloud Specific
    void grabPointCloud(RobWorkStudio* rwstudio){
        // Inspiration from: https://www.robwork.dk/manual/simulated_sensors/
        pcl::PointCloud<PointXYZ>::Ptr pclScene(new pcl::PointCloud<PointXYZ>);
        
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

        pcl::PointCloud<PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZ>);
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
            
            //cloud->points[i].x += noisex;
            //cloud->points[i].y += noisey;
            cloud->points[i].z += noisez;
        }
    }
    
    void calculateSurfaceNormals(){
        // Inspiration from: https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
        
        NormalEstimation<PointXYZ, Normal> ne;
        ne.setInputCloud (cloud);
        
        search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
        ne.setSearchMethod (tree);
        
        pcl::PointCloud<Normal>::Ptr cloud_normals (new pcl::PointCloud<Normal>);
        ne.setRadiusSearch (0.03);
        ne.compute (*cloud_normals);
        
        pcl::PointCloud<PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<PointNormal>);
        concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
        
        *pclNormals = *cloud_with_normals;
    }
    void planeModelSegmentation(){
        // Inspiration from: https://pcl.readthedocs.io/en/latest/planar_segmentation.html
        pcl::PointCloud<PointXYZ>::Ptr planeModel (new pcl::PointCloud<PointXYZ>);
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
        pcl::PointCloud<PointXYZ>::Ptr reducedCloud(new pcl::PointCloud<PointXYZ>);
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
        
        vector<pcl::PointCloud<PointXYZ>::Ptr> objClouds = {};
        for(int i = 0; i < cluster_indices.size(); i++){
            pcl::PointCloud<PointXYZ>::Ptr object(new pcl::PointCloud<PointXYZ>);
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
        
        vector<pcl::PointCloud<PointXYZ>::Ptr> sortedClouds = {};
        // put clouds into new sorted vector
        for (int i = 0; i < N; i++) {
            sortedClouds.push_back(objectClouds[objectIdx[i]]);
        }
        objectClouds = sortedClouds;
    }
    
    vector<PointXYZ> getCentroidsOfClouds(vector<pcl::PointCloud<PointXYZ>::Ptr> clouds){
        vector<PointXYZ> centroids;
        for(auto object : clouds){
            centroids.push_back(centroid(object));
        }
        return centroids;
    }
    vector<Transform3D<> > getObjectsAsTransforms(){
        vector<Transform3D<> > transforms = {};
        vector<PointXYZ> centroids = getCentroidsOfClouds((objectModels.empty() ? objectClouds : objectModels));
        Transform3D<> cameraT = cameraFrame->wTf(wc->getDefaultState());
        for (auto c : centroids) {
            Transform3D<> T(Vector3D<>(c.x, c.y, c.z), RPY<>(0, Deg2Rad*180, 0));
            transforms.push_back(cameraT * T);
        }
        return transforms;
    }
    
    // ICP
    void loadObjectModels(string folder){
        vector<string> filepaths;
        for(auto color : colors) {
            string fp = folder + "SmallerCylinder" + color + ".pcd";
            filepaths.push_back(fp);
        }
        for(auto fp : filepaths) {
            pcl::PointCloud<PointXYZ>::Ptr object(new pcl::PointCloud<PointXYZ>);
            loadPCDFile(fp, *object);
            objectModels.push_back(object);
        }
    }
    void ICP(pcl::PointCloud<PointXYZ>::Ptr objectCloud, pcl::PointCloud<PointXYZ>::Ptr modelCloud, size_t iter = 50){
        // From ex1 in lecture 6 regarding 3D -> 3D pose estimation
        // Create a k-d tree for scene
        search::KdTree<PointXYZ> tree;
        tree.setInputCloud(objectCloud);
        
        // Set ICP parameters
        const float thressq = 0.01 * 0.01;
        
        // Start ICP
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pcl::PointCloud<PointXYZ>::Ptr object_aligned(new pcl::PointCloud<PointXYZ>(*modelCloud));
        {
            ScopeTime t("ICP");
            cout << "Starting ICP..." << endl;
            for(size_t i = 0; i < iter; ++i) {
                // 1) Find closest points
                vector<vector<int> > idx;
                vector<vector<float> > distsq;
                tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
                
                // Threshold and create indices for object/scene and compute RMSE
                vector<int> idxobj;
                vector<int> idxscn;
                for(size_t j = 0; j < idx.size(); ++j) {
                    if(distsq[j][0] <= thressq) {
                        idxobj.push_back(j);
                        idxscn.push_back(idx[j][0]);
                    }
                }
                
                // 2) Estimate transformation
                Eigen::Matrix4f T;
                pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ> est;
                est.estimateRigidTransformation(*object_aligned, idxobj, *objectCloud, idxscn, T);
                
                // 3) Apply pose
                transformPointCloud(*object_aligned, *object_aligned, T);
                
                // 4) Update result
                pose = T * pose;
            }
            
            // Compute inliers and RMSE
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            size_t inliers = 0;
            float rmse = 0;
            for(size_t i = 0; i < distsq.size(); ++i)
                if(distsq[i][0] <= thressq)
                    ++inliers, rmse += distsq[i][0];
            rmse = sqrtf(rmse / inliers);
            
            // Print pose
            cout << "Got the following pose:" << endl << pose << endl;
            cout << "Inliers: " << inliers << "/" << objectCloud->size() << endl;
            cout << "RMSE: " << rmse << endl;
        } // End timing
        

        
        *modelCloud = *object_aligned;
    }
    void moveObjectModels(){
        vector<PointXYZ> objectCentroids = getCentroidsOfClouds(objectClouds);
        vector<PointXYZ> modelCentroids = getCentroidsOfClouds(objectModels);
        
        for (unsigned int i = 0; i < objectModels.size(); i++) {
            PointXYZ vectorObjectModel = sub(modelCentroids[i], objectCentroids[i]);
            translatePointCloud(objectModels[i], vectorObjectModel);
            
            Eigen::Affine3f transform(Eigen::Affine3f::Identity());

            if (i == 0){
                transform.rotate(Eigen::AngleAxisf((60*M_PI) / 180, Eigen::Vector3f::UnitZ()));
                transform.rotate(Eigen::AngleAxisf((60*M_PI) / 180, Eigen::Vector3f::UnitY()));
                transform.rotate(Eigen::AngleAxisf((60*M_PI) / 180, Eigen::Vector3f::UnitX()));
            } else {
                transform.rotate(Eigen::AngleAxisf((120*M_PI) / 180, Eigen::Vector3f::UnitX()));
            }

            Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
            pcl::compute3DCentroid(*objectModels[i], centroid);
            Eigen::Vector4f centroid_new(Eigen::Vector4f::Zero());
            centroid_new.head<3>() = transform.rotation() * centroid.head<3>();
            transform.translation() = centroid.head<3>() - centroid_new.head<3>();
            
            pcl::transformPointCloud(*objectModels[i], *objectModels[i], transform);
            
        }
    }
    void translatePointCloud(pcl::PointCloud<PointXYZ>::Ptr cloudToTranslate, PointXYZ translateVector){
        pcl::PointCloud<PointXYZ>::Ptr translatedCloud(new pcl::PointCloud<PointXYZ>);
        for(auto p : cloudToTranslate->points) {
            PointXYZ translatedPoint = sub(p, translateVector);
            translatedCloud->push_back(translatedPoint);
        }
        *cloudToTranslate = *translatedCloud;
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
    
    PointXYZ centroid(pcl::PointCloud<PointXYZ>::Ptr cloud){
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
    vector<string> colors = {"Red","Green","Blue"};
    double fovy; int width, height;

    pcl::PointCloud<PointXYZ>::Ptr cloud;
    pcl::PointCloud<PointXYZ>::Ptr plane;
    pcl::PointCloud<PointXYZ>::Ptr randomPoints;
    pcl::PointCloud<PointNormal>::Ptr pclNormals;
    ModelCoefficients::Ptr coefficients;
    vector<pcl::PointCloud<PointXYZ>::Ptr> objectClouds;
    vector<pcl::PointCloud<PointXYZ>::Ptr> objectModels;
};

/*
 Ressources:
 https://www.robwork.dk/manual/simulated_sensors/
 https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html?
 https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
 https://pcl.readthedocs.io/en/latest/planar_segmentation.html
 https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html
*/
