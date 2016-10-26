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


registration::registration(){
    std::cout<< "created registration object!\n"<<std::endl;
}

//use constructor initializer list
registration::registration(float downSampleSize, float featureRadius, float maxIterationsSAC) : downSampleSize_(downSampleSize),
                                                                                                featureRadius_(featureRadius),
                                                                                                maxIterationsSAC_(maxIterationsSAC){

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::loadPointClouds(const std::string filename) {

    PCL_INFO("loading *.pcd file...\n");

    pcl::PCDReader reader;

    //create new cloud object on heap
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    int err = reader.read(filename, *cloud);

    if(err){
        PCL_ERROR("could not read *.pcd file\n");
        exit(-1);
    }

    PCL_INFO("successfully loaded point cloud!\n");


    //return (successfully) loaded cloud
    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::filterOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloud) {

    PCL_INFO("filtering outliers from point cloud...\n");

    //create new cloud object on heap
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    //assign cloud
    inCloud = rawCloud;

    //remove outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>::Ptr
            sorfilter = boost::make_shared<pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>>();

    //filter parameters
    sorfilter->setMeanK(50);
    sorfilter->setStddevMulThresh(1.0);
    sorfilter->setInputCloud(inCloud);
    sorfilter->filter(*outCloud);

    PCL_INFO("filtering outliers from point cloud done!\n");

    return outCloud;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                              float downSampleSize) {

    PCL_INFO("begin to voxelize (downsample) point cloud\n");

    pcl::VoxelGrid<pcl::PointXYZRGB> voxGrid;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    inCloud = cloud;

    //leaf size for x, y, z pointcloud coordinates
    voxGrid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);

    voxGrid.setInputCloud(inCloud);
    voxGrid.filter(*outCloud);

    PCL_INFO("voxelization of point cloud done!\n");

    return outCloud;
}



Eigen::Matrix4f registration::registerClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt) {

    PCL_INFO("begin to register point clouds\n");

    //declare transformation matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    //downsample source and target cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_srcCloud = voxelize(src, 0.01);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_tgtCloud = voxelize(tgt, 0.01);

    //compute normals
    pcl::PointCloud<pcl::Normal>::Ptr src_normals = getNormals(ds_srcCloud, src);
    pcl::PointCloud<pcl::Normal>::Ptr tgt_normals = getNormals(ds_tgtCloud, tgt);

    //compute fpfh features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features = getFeaturesFPFH(ds_srcCloud, src_normals, 0.3);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_features = getFeaturesFPFH(ds_tgtCloud, tgt_normals, 0.3);

    //intialize alignment method
    /** TODO: ERROR [pcl::SampleConsensusInitialAlignment::computeTransformation]
     * The target points and target feature points need to be in a one-to-one relationship!
     * Current input cloud sizes: 137304 vs 110090.
     */
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> scia;
    //TODO: needs to be the other way around? src <-> tgt ???
    scia.setInputSource(ds_srcCloud);
    scia.setSourceFeatures(src_features);
    scia.setInputTarget(ds_tgtCloud);
    scia.setTargetFeatures(tgt_features);

    //set parameters for allignment and RANSAC
    scia.setMaxCorrespondenceDistance(0.05);
    scia.setMinSampleDistance(0.5);
    scia.setMaximumIterations(1000);

    //align frame using fpfh features
    scia.align(*ds_tgtCloud);

    //get the new transformation for icp
    transform = scia.getFinalTransformation();


    //icp algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(ds_srcCloud);
    icp.setInputTarget(ds_tgtCloud);

    //set standard icp parameters
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);

    icp.align(*ds_srcCloud);

    std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

    transform = icp.getFinalTransformation() * transform;

    std::cout << "found transformation: " <<std::endl << transform << std::endl;

    return transform;
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr registration::getFeaturesFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                                         double radius) {

    PCL_INFO("begin to detect features of point cloud...\n");

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

    PCL_INFO("feature detection of point cloud done!\n");

    return features;
}


pcl::PointCloud<pcl::Normal>::Ptr registration::getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud) {

    PCL_INFO("begin to determine normals of point cloud...\n");

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud = inCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud = outCloud;

    ne.setInputCloud(inputCloud);
    ne.setSearchSurface(outputCloud);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

    double radius = 0.30;

    ne.setRadiusSearch(radius);
    ne.compute(*normals);

    PCL_INFO("detection of normals from point cloud done!\n");

    return normals;
}


int registration::saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string filename) {

    PCL_INFO("writing point cloud...\n");

    pcl::PCDWriter writer;

    int err = writer.write(filename, *cloud);

    if(err){
        PCL_ERROR("failed to write *.pcd file!\n");
    }
    else{
        PCL_INFO("point cloud written successfully!\n");
    }

    return err;
}

int registration::visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud) {
    //empty for now
    return 0;
}

int registration::initTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename) {

    PCL_INFO("performing initial rotation on X-Axis (PITCH)\n");

    int err = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));

    //transform.rotate(Eigen::AngleAxisf(-(M_PI/2), Eigen::Vector3f::UnitY()));

    pcl::transformPointCloud(*cloud, *outCloud, transform);

    err = pcl::io::savePCDFile(filename, *outCloud);

    PCL_INFO("rotated and written point cloud successfully!\n");

    return err;
}
