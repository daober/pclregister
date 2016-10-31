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
                                     float min_sample_distance, float max_correspondence_distance, int nr_iterations) {


    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr
    sac_ia(new pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>());

    sac_ia->setMinSampleDistance(min_sample_distance);
    sac_ia->setMaxCorrespondenceDistance(max_correspondence_distance);
    sac_ia->setMaximumIterations(nr_iterations);

    sac_ia->setInputCloud(source_points);
    sac_ia->setSourceFeatures(source_descriptors);

    sac_ia->setInputTarget(target_points);
    sac_ia->setTargetFeatures(target_descriptors);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
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


//TODO: METHOD IS FAULTY (Process finished with exit code 139 (interrupted by signal 11: SIGSEGV))
//SIGSEGV = segmentation fault | which in turn means that you tried to access memory that you shouldn't have

Eigen::Matrix4f
Registrator::refineAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                             const Eigen::Matrix4f initial_alignment,
                             float max_correspondence_distance, float outlier_rejection_threshold,
                             float transformation_epsilon, int max_iterations) {


    pcl::console::print_highlight ("Starting alignment...\n");


    /*pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setMaxCorrespondenceDistance(0.1);

    icp.setInputSource(source_points);
    icp.setInputTarget(target_points);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr reg_result = source_points;
    icp.setMaximumIterations(2);

    for (int i = 0; i < 30; ++i){
        PCL_INFO ("Iteration Nr. %d.\n", i);

        //estimation
        icp.setInputSource(source_points);
        icp.align(*reg_result);

        //accumulate transformation between each iteration
        Ti = icp.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if(fabs ((icp.getLastIncrementalTransformation() - prev).sum()) < icp.getTransformationEpsilon()){
            icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() - 0.001);
        }

        prev = icp.getLastIncrementalTransformation();
    }

    //get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // transform target back in source frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outRefined (new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::transformPointCloud(*target_points, *outRefined, targetToSource);

    //add the source to the transformed target
    *outRefined += *source_points;

    //initial_alignment = targetToSource;

    return (icp.getFinalTransformation() * initial_alignment);*/
}


