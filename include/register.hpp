#pragma once

#include <string.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>

#include <Eigen/Geometry>
#include <pcl/keypoints/sift_keypoint.h>


class registration {

public:

    /** simple structure for storing all of a cloud's features */
    struct ObjectFeatures
    {
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

    boost::shared_ptr<registration::ObjectFeatures>
    computeFeatures (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input);

/** Use a PassThrough filter to remove points with depth values that are too large or too small */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    thresholdDepth (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                    float min_depth,
                    float max_depth);

/** Use a VoxelGrid filter to reduce the number of points */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                                                    float leaf_size);

/** Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    removeOutliers (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                                                           float radius,
                                                           int min_neighbors);

/** Apply a series of filters (threshold depth, downsample, and remove outliers) */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    applyFilters (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                  float min_depth,
                  float max_depth,
                  float leaf_size,
                  float radius,
                  float min_neighbors);



    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
    loadPointCloud (std::string filename, std::string suffix);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    loadPoints (std::string filename);

    pcl::PointCloud<pcl::Normal>::Ptr
    loadSurfaceNormals(std::string filename);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    loadKeypoints (std::string filename);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
    loadLocalDescriptors (std::string filename);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr
    loadGlobalDescriptors (std::string filename);


    /** Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
 * correspondences between two sets of local features
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   source_descriptors
 *     The local descriptors for each source point
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   target_descriptors
 *     The local descriptors for each target point
 *   min_sample_distance
 *     The minimum distance between any two random samples
 *   max_correspondence_distance
 *     The
 *   nr_interations
 *     The number of RANSAC iterations to perform
 * Return: A transformation matrix that will roughly align the points in source to the points in target
 */
    Eigen::Matrix4f
    computeInitialAlignment (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & source_points,
                             const pcl::PointCloud<pcl::FPFHSignature33>::Ptr & source_descriptors,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & target_points,
                             const pcl::PointCloud<pcl::FPFHSignature33>::Ptr & target_descriptors,
                             float min_sample_distance, float max_correspondence_distance, int nr_iterations);


/** Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
 * starting with an intial guess
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   intial_alignment
 *     An initial estimate of the transformation matrix that aligns the source points to the target points
 *   max_correspondence_distance
 *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further
 *     apart than this threshold will be ignored when computing the source-to-target transformation
 *   outlier_rejection_threshold
 *     A threshold used to define outliers during RANSAC outlier rejection
 *   transformation_epsilon
 *     The smallest iterative transformation allowed before the algorithm is considered to have converged
 *   max_iterations
 *     The maximum number of ICP iterations to perform
 * Return: A transformation matrix that will precisely align the points in source to the points in target
 */
    Eigen::Matrix4f
    refineAlignment (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & source_points,
                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & target_points,
                     const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                     float outlier_rejection_threshold, float transformation_epsilon, float max_iterations);


private:



};