
//include own headers first
#include "register.hpp"

#include <pcl/io/boost.h>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>



int main(int argc, char **argv){

    //error code
    int err = 0;

    //initialize class object
    boost::shared_ptr<registration> registrator = boost::make_shared<registration>();

    if (argc < 3) {
        pcl::console::print_info ("Syntax is: %s source target <options>\n", argv[0]);
        pcl::console::print_info ("  where options are:\n");
        pcl::console::print_info ("    -i min_sample_dist,max_dist,nr_iters ................ Compute initial alignment\n");
        pcl::console::print_info ("    -r max_dist,rejection_thresh,tform_eps,max_iters ............. Refine alignment\n");
        pcl::console::print_info ("    -s output.pcd ........................... Save the registered and merged clouds\n");
        pcl::console::print_info ("Note: The inputs (source and target) must be specified without the .pcd extension\n");

        return (1);
    }

    // Load the pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_points = registrator->loadPoints (argv[1]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_points = registrator->loadPoints (argv[2]);

    // Compute the intial alignment
    double min_sample_dist;
    double max_correspondence_dist;
    double nr_iters;


    
    return (0);
}