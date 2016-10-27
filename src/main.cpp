
//include own headers first
#include "register.hpp"

#include <pcl/io/boost.h>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>



int main(int argc, char **argv){

    //error code
    int err = 0;

    //create new class object as smart pointer
    boost::shared_ptr<registration> reg = boost::make_shared<registration>();

    //Create output directory
    if(boost::filesystem::exists("aligned")) {
        std::cout << "directory already exists, doing nothing." << std::endl;
        err = -0;
    }
    else if(!boost::filesystem::create_directory("aligned")){
        std::cout<<"Error! Could not create output directory."<<std::endl;
        err = -1;
    }
    else{
        std::cout<<"directory created"<<std::endl;
        err = 0;
    }


    //load point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src = reg->loadPointClouds("room1.pcd");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt = reg->loadPointClouds("room2.pcd");

    //initial (rotation)
    reg->initRotation(src, "aligned/room1.pcd");
    reg->initRotation(tgt, "aligned/room2.pcd");

    //create temporary point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempSrcCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempTgtCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    //do the registration and update the transformation
    Eigen::Matrix4f transform = reg->registerClouds(tgt, src, true, false);

    return (0);
}