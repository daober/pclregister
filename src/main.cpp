
//include own headers first
#include "register.hpp"

#include <iostream>
#include <pcl/io/boost.h>
#include <boost/make_shared.hpp>



int main(int argc, char **argv){
    
    //create new class object as smart pointer
    boost::shared_ptr<registration> reg = boost::make_shared<registration>();

    //Create output directory
    if(!boost::filesystem::create_directory("aligned")) {
        std::cout << "Error! Could not create output directory." << std::endl;
        return (-1);
    }

    //load point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src = reg->loadPointClouds("room_01.pcd");
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt = reg->loadPointClouds("room_03.pcd");

    //initialize transformation from points in frame to points in model
    Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();



    return (0);
}