#include "registrator.hpp"


//this file is needed for succesful linking on *unix machines
#include <pcl/search/impl/search.hpp>

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_rejection_features.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string.h>




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
Registrator::computeInitialAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                                    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                                    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
                                    float min_sample_distance,
                                    float max_correspondence_distance,
                                    int nr_iterations) {


    pcl::console::print_highlight ("starting initial alignment...\n");

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr
    sac_ia (new pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>());


    sac_ia->setMinSampleDistance(min_sample_distance);
    sac_ia->setMaxCorrespondenceDistance(max_correspondence_distance);
    sac_ia->setMaximumIterations(nr_iterations);

    sac_ia->setInputSource(source_points);
    sac_ia->setSourceFeatures(source_descriptors);

    sac_ia->setInputTarget(target_points);
    sac_ia->setTargetFeatures(target_descriptors);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
            outCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    sac_ia->align(*outCloud);

    return (sac_ia->getFinalTransformation());
}


/** Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
 * starting with an initial guess
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   initial_alignment
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
Registrator::refineAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                             const Eigen::Matrix4f &initial_alignment,
                             float max_correspondence_distance,
                             float outlier_rejection_threshold,
                             float transformation_epsilon,
                             int max_iterations) {


    pcl::console::print_highlight ("starting refined alignment...\n");

    pcl::PointCloud<pcl::PointXYZRGB> register_output;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr
            icp (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());


    icp->setMaxCorrespondenceDistance(max_correspondence_distance);
    icp->setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
    icp->setTransformationEpsilon(transformation_epsilon);
    icp->setMaximumIterations(max_iterations);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
            sourcePointsTransformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    pcl::transformPointCloud(*source_points, *sourcePointsTransformed, initial_alignment);

    icp->setInputSource(sourcePointsTransformed);
    icp->setInputTarget(target_points);

    icp->align(register_output);

    return (icp->getFinalTransformation() * initial_alignment);
}


