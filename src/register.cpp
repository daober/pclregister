#include "register.hpp"


//this file is needed for succesful linking on *unix machines
#include <pcl/search/impl/search.hpp>

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/filters/filter.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
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
#include <pcl/registration/correspondence_rejection_features.h>

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

    pcl::PCDReader reader;

    //create new cloud object on heap
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    int err = reader.read(filename, *cloud);

    if(err){
        PCL_ERROR("could not read *.pcd file\n");
        exit(-1);
    }

    //filter outliers and return filtered cloud
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud = filterOutliers(cloud);

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
            sor = boost::make_shared<pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>>();

    //filter parameters
    sor->setMeanK(50);
    sor->setStddevMulThresh(1.0);
    sor->setInputCloud(inCloud);
    sor->filter(*outCloud);

    PCL_INFO("filtering outliers from point cloud done!\n");

    return outCloud;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                              float downSampleSize) {

    PCL_INFO("begin to voxelize (downsample) point cloud\n");

    pcl::VoxelGrid<pcl::PointXYZRGB> voxGrid;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    //assign clouds
    inCloud = cloud;

    std::cout<<"point clouds before filtering: " << inCloud->width * inCloud->height << std::endl;

    //leaf size for x, y, z pointcloud coordinates
    voxGrid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);

    voxGrid.setInputCloud(inCloud);
    voxGrid.filter(*outCloud);

    std::cout<<"point clouds after filtering: " << outCloud->width * outCloud->height << std::endl;

    PCL_INFO("voxelization of point cloud done!\n");

    return outCloud;
}



Eigen::Matrix4f registration::registerClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, bool useFPFH, bool useICP) {

    PCL_INFO("begin to register point clouds\n");

    //declare transformation matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    //downsample source and target cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_srcCloud = voxelize(src, 0.01);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_tgtCloud = voxelize(tgt, 0.01);

    if(useFPFH){
        //TODO: visualize features
        std::cout<< "FPFH estimation selected" << std::endl;

        //compute normals
        pcl::PointCloud<pcl::Normal>::Ptr src_normals = getNormals(ds_srcCloud, src);
        pcl::PointCloud<pcl::Normal>::Ptr tgt_normals = getNormals(ds_tgtCloud, tgt);

        //compute fpfh features
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features = getFeaturesFPFH(ds_srcCloud, src_normals, 0.2);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_features = getFeaturesFPFH(ds_tgtCloud, tgt_normals, 0.2);

        //get refined interest points
        pcl::Correspondences corr = estimateCorrespondences(ds_tgtCloud, ds_srcCloud, tgt_features, src_features);

        //merge point clouds into global model with transformation for alignment
        //TODO: this function needs to be looped!!!
        transform = mergeClouds(ds_tgtCloud, ds_srcCloud, transform, tgt_features, src_features, corr);

    }

    //TODO: ICP not used for now!!!
    if(useICP){

        std::cout<< "icp selected" << std::endl;

        //icp algorithm
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(ds_tgtCloud);
        icp.setInputTarget(ds_srcCloud);

        //set standard icp parameters
        icp.setMaxCorrespondenceDistance(0.2);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-5);

        icp.align(*ds_tgtCloud);

        std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

        transform = icp.getFinalTransformation() * transform;
    }

    std::cout << "found transformation: " <<std::endl << transform << std::endl;

    //transform point cloud according to calculated transformation values
    pcl::transformPointCloud(*ds_tgtCloud, *ds_tgtCloud, transform);

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

    std::cout<< "number of interest points: "<< features->width * features->height <<std::endl;

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

    double radius = 0.1;        //radius of 1cm
    ne.setRadiusSearch(radius);

    //compute features
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


int registration::initRotation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename) {

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



pcl::PointCloud<pcl::PointWithScale>::Ptr registration::getSIFTKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale>::Ptr
            sift (new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale>());


    pcl::PointCloud<pcl::PointWithScale>::Ptr result (new pcl::PointCloud<pcl::PointWithScale> ());

    //use FLANN-based KdTree to perform neighborhood searches Format PointWithScale(x, y, z, scale)
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());

    sift->setSearchMethod(tree);

    const float min_scale = 0.1f;
    const float min_contrast = 0.1f;
    const int nr_octaves = 8;
    const int scales_per_octave = 16;

    //set sift input parameters
    sift->setScales(min_scale, nr_octaves, scales_per_octave);
    sift->setMinimumContrast(min_contrast);
    sift->setInputCloud(cloud);
    sift->compute(*result);

    std::cout<<"result of sift (size) source: " << result->points.size() << std::endl;

    return result;
}



pcl::Correspondences registration::estimateCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt,
                                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgtfeat,
                                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr srcfeat) {


    pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr corr_est
            (new pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>());

    //holds correspondences between source and target clouds
    boost::shared_ptr<pcl::Correspondences> corr (new pcl::Correspondences());

    corr_est->setInputSource(src);
    corr_est->setInputTarget(tgt);

    corr_est->determineReciprocalCorrespondences(*corr);
    std::cout<<"correspondences between target and source are: " << corr->size() <<std::endl;

    //TODO: reject bad correspondences
     pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB>::Ptr corr_reject
             (new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB>());


    //boost::shared_ptr<pcl::Correspondences> corr_remain (new pcl::Correspondences());

    //corr_reject->setInputSource();
    //corr_reject.setTargetFeature(tgtfeat, "target");

    //corr_reject.setInputCorrespondences(corr);
    //corr_reject.getCorrespondences(*corr);

    //std::cout<<"correspondences after rejection are: " << corr->size() <<std::endl;

    return *corr;
}

Eigen::Matrix4f registration::mergeClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                                          Eigen::Matrix4f &transform,
                                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgtfeat,
                                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr srcfeat,
                                          pcl::Correspondences &corr) {


    std::cout<< "begin to merge clouds..." <<std::endl;

    //initial alignment
    /*pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> scia;

    scia.setInputSource(tgt);
    scia.setSourceFeatures(tgtfeat);
    scia.setInputTarget(src);
    scia.setTargetFeatures(srcfeat);

    //set parameters for alignment and RANSAC
    scia.setMaxCorrespondenceDistance(0.50);
    scia.setMinSampleDistance(0.5);
    scia.setMaximumIterations(1000);

    //align frame using fpfh features
    scia.align(*tgt);*/

    //Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();

    //estimate rigid transformation
    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr
    //        est_trans (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB>());


    //est_trans->estimateRigidTransformation(*tgt, *src, corr, transform);
    //pcl::transformPointCloud(*src, *tgt, transform);

    //get the new transformation for icp
    //transform = scia.getFinalTransformation();

    //current_transform = transform * current_transform;

    //merge point clouds into global model
    *src += *tgt;

    //save merged point cloud as pcd file
    pcl::io::savePCDFile("aligned/finalOutput.pcd", *src);

    std::cout<< "end of merging..." <<std::endl;

    return transform;
}
