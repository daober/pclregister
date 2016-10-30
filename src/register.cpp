#include "register.hpp"


//this file is needed for succesful linking on *unix machines
#include <pcl/search/impl/search.hpp>

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/features/vfh.h>
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
#include <pcl/registration/transformation_estimation_svd.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string.h>


pcl::PointCloud<pcl::Normal>::Ptr
registration::estimateSurfaceNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, float radius) {

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    normalEstimation.setSearchMethod(tree);
    normalEstimation.setRadiusSearch(radius);
    normalEstimation.setInputCloud(inputCloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

    //compute estimated normals of point cloud and return them
    normalEstimation.compute(*normals);

    return (normals);


}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::detectKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                              const pcl::PointCloud<pcl::Normal>::Ptr &normals, float min_scale, int nr_octaves,
                              int nr_scales_per_octave, float min_contrast) {

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale>::Ptr sift (new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale>());

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());

    sift->setSearchMethod(tree);
    sift->setScales(min_scale, nr_octaves, nr_scales_per_octave);
    sift->setMinimumContrast(min_contrast);
    sift->setInputCloud(points);

    pcl::PointCloud<pcl::PointWithScale> tempKeypoints;
    //compute the keypoints
    sift->compute(tempKeypoints);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(tempKeypoints, *keypoints);

    return (keypoints);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
registration::computeLocalDescriptors(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                      const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints, float feature_radius) {

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation
            (new pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>());

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());

    fpfh_estimation->setSearchMethod(tree);
    fpfh_estimation->setRadiusSearch(feature_radius);
    fpfh_estimation->setSearchSurface(points);
    fpfh_estimation->setInputNormals(normals);
    fpfh_estimation->setInputCloud(keypoints);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr localDesc (new pcl::PointCloud<pcl::FPFHSignature33>());

    fpfh_estimation->compute(*localDesc);

    return (localDesc);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr
registration::computeGlobalDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                      const pcl::PointCloud<pcl::Normal>::Ptr &normals) {

    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308>::Ptr
            vfh_estimation (new pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308>());

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());

    vfh_estimation->setSearchMethod(tree);
    vfh_estimation->setInputCloud(points);
    vfh_estimation->setInputNormals(normals);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr globalDesc (new pcl::PointCloud<pcl::VFHSignature308>());

    return (globalDesc);
}



boost::shared_ptr<registration::ObjectFeatures>
registration::computeFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input) {

    boost::shared_ptr<ObjectFeatures> features = boost::make_shared<ObjectFeatures>();

    features->points = input;
    features->normals = estimateSurfaceNormals(input, 0.05);
    features->keypoints = detectKeypoints(input, features->normals, 0.005, 10, 8, 1.5);
    features->local_descriptors = computeLocalDescriptors(input, features->normals, features->keypoints, 0.1);
    features->global_descriptor = computeGlobalDescriptor(input, features->normals);
    
    return features;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::thresholdDepth(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float min_depth, float max_depth) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float leaf_size) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float radius, int min_neighbors) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::applyFilters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float min_depth, float max_depth,
                           float leaf_size, float radius, float min_neighbors) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
registration::loadPointCloud(std::string filename, std::string suffix) {
    return boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::loadPoints(std::string filename) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr registration::loadGlobalDescriptors(std::string filename) {
    return pcl::PointCloud<pcl::VFHSignature308>::Ptr();
}

pcl::PointCloud<pcl::Normal>::Ptr registration::loadSurfaceNormals(std::string filename) {
    return pcl::PointCloud<pcl::Normal>::Ptr();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::loadKeypoints(std::string filename) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr registration::loadLocalDescriptors(std::string filename) {
    return pcl::PointCloud<pcl::FPFHSignature33>::Ptr();
}

Eigen::Matrix4f registration::computeInitialAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                                                      const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
                                                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                                                      const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
                                                      float min_sample_distance, float max_correspondence_distance,
                                                      int nr_iterations) {
    return Eigen::Matrix4f();
}

Eigen::Matrix4f registration::refineAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                                              const Eigen::Matrix4f initial_alignment,
                                              float max_correspondence_distance, float outlier_rejection_threshold,
                                              float transformation_epsilon, float max_iterations) {
    return Eigen::Matrix4f();
}


