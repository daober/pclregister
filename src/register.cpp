#include "register.hpp"


//this file is needed for succesful linking on *unix machines
#include <pcl/search/impl/search.hpp>

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>

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

    pcl::PassThrough<pcl::PointXYZRGB>::Ptr passThrough (new pcl::PassThrough<pcl::PointXYZRGB>());

    passThrough->setInputCloud(input);
    passThrough->setFilterFieldName("z");
    passThrough->setFilterLimits(min_depth, max_depth);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr threshold (new pcl::PointCloud<pcl::PointXYZRGB>());
    passThrough->filter(*threshold);

    return (threshold);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float leaf_size) {

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud (input);
    voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel_grid.filter (*downsampled);

    return (downsampled);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float radius, int min_neighbors) {

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
    radius_outlier_removal.setInputCloud (input);
    radius_outlier_removal.setRadiusSearch (radius);
    radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers (new pcl::PointCloud<pcl::PointXYZRGB>());
    radius_outlier_removal.filter (*inliers);

    return (inliers);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
registration::applyFilters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float min_depth, float max_depth,
                           float leaf_size, float radius, float min_neighbors) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter (new pcl::PointCloud<pcl::PointXYZRGB>());

    filter = thresholdDepth(input, min_depth, max_depth);
    filter = voxelize(filter, leaf_size);
    filter = removeOutliers(filter, radius, min_neighbors);

    return (filter);
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
registration::loadPointCloud(std::string filename, std::string suffix) {

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > output (new pcl::PointCloud<pcl::PointXYZRGB>);
    filename.append (suffix);

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("Loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::loadPoints(std::string filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>());
    filename.append ("_points.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("Loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr registration::loadLocalDescriptors(std::string filename) {
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr output (new pcl::PointCloud<pcl::FPFHSignature33>());
    filename.append ("_localdesc.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("Loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr registration::loadGlobalDescriptors(std::string filename) {
    pcl::PointCloud<pcl::VFHSignature308>::Ptr output (new pcl::PointCloud<pcl::VFHSignature308>());
    filename.append ("_globaldesc.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("Loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}

pcl::PointCloud<pcl::Normal>::Ptr registration::loadSurfaceNormals(std::string filename) {
    pcl::PointCloud<pcl::Normal>::Ptr output (new pcl::PointCloud<pcl::Normal>());
    filename.append ("_normals.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("Loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::loadKeypoints(std::string filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>());
    filename.append ("_keypoints.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("Loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}


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
registration::computeInitialAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                                                      const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
                                                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                                                      const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
                                                      float min_sample_distance, float max_correspondence_distance,
                                                      int nr_iterations) {


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

Eigen::Matrix4f
registration::refineAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                                              const Eigen::Matrix4f initial_alignment,
                                              float max_correspondence_distance, float outlier_rejection_threshold,
                                              float transformation_epsilon, float max_iterations) {

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());

    icp->setMaxCorrespondenceDistance(max_correspondence_distance);
    icp->setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
    icp->setTransformationEpsilon(transformation_epsilon);
    icp->setMaximumIterations(max_iterations);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outSource (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*source_points, *outSource, initial_alignment);

    icp->setInputCloud(outSource);
    icp->setInputTarget(target_points);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outRefined(new pcl::PointCloud<pcl::PointXYZRGB>());

    icp->align(*outRefined);

    return (icp->getFinalTransformation() * initial_alignment);
}


