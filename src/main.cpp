
//include own headers first
#include "register.hpp"

#include <pcl/io/boost.h>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>



int main(int argc, char **argv){

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

    //TODO: ITERATION is necessary!!!!
    //load point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src = reg->loadPointClouds("room1.pcd");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt = reg->loadPointClouds("room2.pcd");

    //initial transformation
    reg->initTransform(src, "aligned/room1.pcd");
    reg->initTransform(tgt, "aligned/room2.pcd");

    //create temporary point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempSrcCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempTgtCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    //sample down point clouds
    tempSrcCloud = reg->voxelize(tempSrcCloud, 0.15f);
    tempTgtCloud = reg->voxelize(tempTgtCloud, 0.15f);

    //initialize transformation from points in frame to points in model
    Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();

    //for(int i = 0; i < 30; ++i){
        //PCL_INFO ("Iteration Nr. %d.\n", i);

        pcl::transformPointCloud(*tgt, *tempTgtCloud, current_transform);
        pcl::transformPointCloud(*src, *tempSrcCloud, current_transform);

        //do the registration and update the transformation
        Eigen::Matrix4f transform = reg->registerClouds(tempTgtCloud, tempSrcCloud, true, false);
        current_transform = transform * current_transform;

        //add target cloud to source cloud
        *tempSrcCloud += *tempTgtCloud;

   // }

    reg->saveCloud(tempSrcCloud, "aligned/outputSrc.pcd");
    reg->saveCloud(tempTgtCloud, "aligned/outputTgt.pcd");

    return (0);
}