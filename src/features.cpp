#include "features.hpp"

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>

#include <pcl/search/impl/search.hpp>
#include <boost/shared_ptr.hpp>



pcl::PointCloud<pcl::Normal>::Ptr
Features::estimateSurfaceNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, float radius) {

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
Features::detectKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
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
Features::computeLocalDescriptors(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
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
Features::computeGlobalDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
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



boost::shared_ptr<Features::ObjectFeatures>
Features::computeFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input) {

    boost::shared_ptr<ObjectFeatures> features = boost::make_shared<ObjectFeatures>();

    features->points = input;
    features->normals = estimateSurfaceNormals(input, 0.05);
    features->keypoints = detectKeypoints(input, features->normals, 0.005, 10, 8, 1.5);
    features->local_descriptors = computeLocalDescriptors(input, features->normals, features->keypoints, 0.1);
    features->global_descriptor = computeGlobalDescriptor(input, features->normals);

    return features;
}