
//include own headers first
#include "registrator.hpp"

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

    if (argc < 3) {
        pcl::console::print_info ("Syntax is: %s source target <options>\n", argv[0]);
        pcl::console::print_info ("  where options are:\n");
        pcl::console::print_info ("    -i min_sample_dist,max_dist,nr_iters ................ Compute initial alignment\n");
        pcl::console::print_info ("    -r max_dist,rejection_thresh,tform_eps,max_iters ............. Refine alignment\n");
        pcl::console::print_info ("    -s output.pcd ........................... Save the registered and merged clouds\n");
        pcl::console::print_info ("Note: The inputs (source and target) must be specified without the .pcd extension\n");

        return (1);
    }

    // load the pointclouds
    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_points = registrator->loadPoints (argv[1]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_points = registrator->loadPoints (argv[2]);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity ();

    // compute the intial alignment
    double min_sample_dist;
    double max_correspondence_dist;
    double nr_iters;

    bool initialAlignment = pcl::console::parse_3x_arguments
                                    ( argc, argv, "-i", min_sample_dist,  max_correspondence_dist, nr_iters) > 0;

    if(initialAlignment){
        // load the keypoints and local descriptors
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr srcKeypoints = registrator->loadKeypoints(argv[1]);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr srcDescriptor = registrator->loadLocalDescriptors(argv[1]);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgtKeypoints = registrator->loadKeypoints(argv[2]);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgtDescriptor = registrator->loadLocalDescriptors(argv[2]);

        // find the transform that roughly aligns the points
        transform = registrator->computeInitialAlignment(srcKeypoints,
                                                         srcDescriptor,
                                                         tgtKeypoints,
                                                         tgtDescriptor,
                                                         min_sample_dist,
                                                         max_correspondence_dist,
                                                         nr_iters);

        pcl::console::print_info ("computed initial alignment!\n");
    }

    //refine the result of initial alignment
    std::string params_string;

    bool refineAlignment = pcl::console::parse_argument (argc, argv, "-r", params_string) > 0;

    if (refineAlignment){
        std::vector<std::string> tokens;

        boost::split (tokens, params_string, boost::is_any_of (","), boost::token_compress_on);

        assert (tokens.size () == 4);
        float max_correspondence_distance = atof(tokens[0].c_str ());
        float outlier_rejection_threshold = atof(tokens[1].c_str ());
        float transformation_epsilon = atoi(tokens[2].c_str ());
        int max_iterations = atoi(tokens[3].c_str ());

        transform = registrator->refineAlignment (src_points,
                                                  tgt_points,
                                                  transform,
                                                  max_correspondence_distance,
                                                  outlier_rejection_threshold,
                                                  transformation_epsilon,
                                                  max_iterations);

        pcl::console::print_info ("refined alignment!\n");
    }

    // transform the source point to align them with the target points
    pcl::transformPointCloud (*src_points, *src_points, transform);

    // save output
    std::string filename;
    bool saveOutput = pcl::console::parse_argument (argc, argv, "-s", filename) > 0;

    if (saveOutput){
        // merge the two clouds
        (*src_points) += (*tgt_points);

        // save the result
        pcl::io::savePCDFile (filename, *src_points);
        pcl::console::print_info ("saved registered clouds as %s\n", filename.c_str ());
    }
    // or visualize "on the fly" via visualizer (vtk)
    else{
        pcl::console::print_info ("starting visualizer... close window to exit\n");
        pcl::visualization::PCLVisualizer vis;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (src_points, 255, 0, 0);
        vis.addPointCloud (src_points, red, "src_points");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> yellow (tgt_points, 255, 255, 0);
        vis.addPointCloud (tgt_points, yellow, "tgt_points");

        vis.resetCamera ();
        vis.spin ();
    }*/

    return (0);
}