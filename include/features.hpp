#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
 *
 * This class is used solely for feature detection of various point clouds (e.g. interest points)
 *
 * */
class Features{

public:


    /** simple structure for storing all of a cloud's features */
    struct ObjectFeatures{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr local_descriptors;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptor;
    };



    /** Use NormalEstimation to estimate a cloud's surface normals
 * Inputs:
 *   input
 *     The input point cloud
 *   radius
 *     The size of the local neighborhood used to estimate the surface
 * Return: A pointer to a SurfaceNormals point cloud
 */
    pcl::PointCloud<pcl::Normal>::Ptr
    estimateSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, float radius);

/** Use SIFTKeypoint to detect a set of keypoints
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   min_scale
 *     The smallest scale in the difference-of-Gaussians (DoG) scale-space
 *   nr_octaves
 *     The number of times the scale doubles in the DoG scale-space
 *   nr_scales_per_octave
 *     The number of scales computed for each doubling
 *   min_contrast
 *     The minimum local contrast that must be present for a keypoint to be detected
 * Return: A pointer to a point cloud of keypoints
 */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    detectKeypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & points,
                     const pcl::PointCloud<pcl::Normal>::Ptr & normals,
                     float min_scale,
                     int nr_octaves,
                     int nr_scales_per_octave,
                     float min_contrast);

/** Use FPFHEstimation to compute local feature descriptors around each keypoint
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   keypoints
 *     A cloud of keypoints specifying the positions at which the descriptors should be computed
 *   feature_radius
 *     The size of the neighborhood from which the local descriptors will be computed
 * Return: A pointer to a LocalDescriptors (a cloud of LocalDescriptorT points)
 */

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
    computeLocalDescriptors(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                            const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
                            float feature_radius);

/** Use VFHEstimation to compute a single global descriptor for the entire input cloud
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 * Return: A pointer to a GlobalDescriptors point cloud (a cloud containing a single GlobalDescriptorT point)
 */

    pcl::PointCloud<pcl::VFHSignature308>::Ptr
    computeGlobalDescriptor (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & points,
                             const pcl::PointCloud<pcl::Normal>::Ptr &normals);


/** Estimate normals, detect keypoints, and compute local and global descriptors
 * Return: An ObjectFeatures struct containing all the features
 */
    boost::shared_ptr<Features::ObjectFeatures>
    computeFeatures (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input);




private:

};