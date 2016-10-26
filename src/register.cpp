#include "register.hpp"


//this file is needed for succesful linking on *unix machines
#include <pcl/search/impl/search.hpp>

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/transformation_estimation.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string.h>




//use constructor initializer list
registration::registration(float downSampleSize, float featureRadius, float maxIterationsSAC) : downSampleSize_(downSampleSize),
                                                                                                featureRadius_(featureRadius),
                                                                                                maxIterationsSAC_(maxIterationsSAC){

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::loadPointClouds(const std::string filename) {

    PCL_INFO("loading *.pcd file...");

    pcl::PCDReader reader;

    //create new cloud object on heap
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    int err = reader.read(filename, *cloud);

    if(err){
        PCL_ERROR("could not read *.pcd file");
        exit(-1);
    }

    PCL_INFO("successfully loaded point cloud!");

    //return (successfully) loaded cloud
    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::filterOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloud) {

    PCL_INFO("filtering outliers from point cloud...");

    //create new cloud object on heap
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    //assign cloud
    inCloud = rawCloud;

    //remove outliers
    //TODO: need to change parameters for own cloud
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>::Ptr
            sorfilter = boost::make_shared<pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>>();

    //filter parameters
    //TODO: needs to refined

    sorfilter->setMeanK(50);
    sorfilter->setStddevMulThresh(1.0);
    sorfilter->setInputCloud(inCloud);
    sorfilter->filter(*outCloud);

    PCL_INFO("filtering outliers from point cloud done!");

    return outCloud;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                              float downSampleSize) {

    PCL_INFO("begin to voxelize (downsample) point cloud");

    pcl::VoxelGrid<pcl::PointXYZRGB> voxGrid;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    inCloud = cloud;

    //leaf size for x, y, z pointcloud coordinates
    voxGrid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);

    voxGrid.setInputCloud(inCloud);
    voxGrid.filter(*outCloud);

    PCL_INFO("voxelization of point cloud done!");

    return outCloud;
}



Eigen::Matrix4f registration::registerClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {


}



pcl::PointCloud<pcl::FPFHSignature33>::Ptr registration::getFeaturesFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                                         double radius) {

    PCL_INFO("begin to detect features of point cloud...");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::Normal>::Ptr inNormals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

    //assign pointclouds
    inCloud = cloud;
    inNormals = normals;

    //create new feature signature for FPFH algorithm
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr feat_tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

    fpfh_est.setInputCloud(inCloud);
    fpfh_est.setInputNormals(inNormals);
    fpfh_est.setSearchMethod(feat_tree);
    fpfh_est.setRadiusSearch(radius);
    fpfh_est.compute(*features);

    PCL_INFO("feature detection of point cloud done!");

    return features;
}



pcl::PointCloud<pcl::Normal>::Ptr registration::getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud) {

    PCL_INFO("begin to determine normals of point cloud...");

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud = inCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud = outCloud;

    ne.setInputCloud(inputCloud);
    ne.setSearchSurface(outputCloud);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

    double radius = 3.00;

    ne.setRadiusSearch(radius);
    ne.compute(*normals);

    PCL_INFO("detection of normals from point cloud done!");

    return normals;
}


int registration::saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string filename) {

    PCL_INFO("writing point cloud...");

    pcl::PCDWriter writer;

    int err = writer.write(filename, *cloud);

    if(err){
        PCL_ERROR("failed to write *.pcd file!");
    }
    else{
        PCL_INFO("point cloud written successfully!");
    }

    return err;
}