//include own headers first
#include "registrator.hpp"
#include "filters.hpp"
#include "features.hpp"
#include "loader.hpp"

#include <pcl/io/boost.h>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>

/*for additional parsing options*/
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char **argv){

    //error code
    int err = 0;

    //initialize class object
    boost::shared_ptr<Registrator> registrator = boost::make_shared<Registrator>();
    boost::shared_ptr<Filters> filter = boost::make_shared<Filters>();
    boost::shared_ptr<Loader> loader = boost::make_shared<Loader>();
    boost::shared_ptr<Saver> saver = boost::make_shared<Saver>();
    boost::shared_ptr<Features> feature = boost::make_shared<Features>();

    // load the pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_points = loader->loadPoints ("room1");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_points = loader->loadPoints ("room2");

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity ();

    boost::shared_ptr<Features::ObjectFeatures> srcFeatures = boost::make_shared<Features::ObjectFeatures>();
    boost::shared_ptr<Features::ObjectFeatures> tgtFeatures = boost::make_shared<Features::ObjectFeatures>();

    srcFeatures = feature->computeFeatures(src_points);
    tgtFeatures = feature->computeFeatures(tgt_points);

    saver->savePoints("room1", srcFeatures->points);
    saver->saveKeypoints("room1", srcFeatures->keypoints);
    saver->saveSurfaceNormals("room1", srcFeatures->normals);
    saver->saveLocalDescriptors("room1", srcFeatures->local_descriptors);

    saver->savePoints("room2", tgtFeatures->points);
    saver->saveKeypoints("room2", tgtFeatures->keypoints);
    saver->saveSurfaceNormals("room2", tgtFeatures->normals);
    saver->saveLocalDescriptors("room2", tgtFeatures->local_descriptors);


    // compute the initial alignment
    double min_sample_dist = 1e-6;
    double max_correspondence_dist = 0.03f;
    double nr_iters = 5000;

    // load the keypoints and local descriptors
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr srcKeypoints = loader->loadKeypoints("room1");
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr srcDescriptor = loader->loadLocalDescriptors("room1");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgtKeypoints = loader->loadKeypoints("room2");
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgtDescriptor = loader->loadLocalDescriptors("room2");

    // find the transform that roughly aligns the points
    transform = registrator->computeInitialAlignment(srcKeypoints, srcDescriptor, tgtKeypoints, tgtDescriptor,
                                                     min_sample_dist, max_correspondence_dist, nr_iters);

    pcl::console::print_info ("computed initial alignment!\n");


    float max_correspondence_distance = 0.01f;
    float outlier_rejection_threshold = 0.50f;
    float transformation_epsilon = 1e-6;
    int max_iterations = 50;
    Eigen::Matrix4f tempTrans = Eigen::Matrix4f::Identity();


    //TODO: this method is faulty SIGILL (4)
    //transform = registrator->refineAlignment (src_points, tgt_points, transform, max_correspondence_distance,
    //                                          outlier_rejection_threshold, transformation_epsilon, max_iterations);

    pcl::console::print_info ("refined alignment!\n");

    // transform the source point to align them with the target points
    pcl::transformPointCloud (*src_points, *src_points, transform);

    // save output
    std::string filename("output.pcd");

    // merge the two clouds
    (*src_points) += (*tgt_points);

    // save the result
    pcl::io::savePCDFile (filename, *src_points);
    pcl::console::print_info ("saved registered clouds as %s\n", filename.c_str ());

    //and visualize the result via vtk

    /*pcl::console::print_info ("starting visualizer... close window to exit\n");
    pcl::visualization::PCLVisualizer vis;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (src_points, 255, 0, 0);
    vis.addPointCloud (src_points, red, "src_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> yellow (tgt_points, 255, 255, 0);
    vis.addPointCloud (tgt_points, yellow, "tgt_points");

    vis.resetCamera ();

    while(!vis.wasStopped()){
        vis.spinOnce();
    }*/


    return (0);
}