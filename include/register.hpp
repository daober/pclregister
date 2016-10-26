#pragma once

#include <string.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/fpfh.h>

#include <Eigen/Geometry>

class registration {

public:
    //constructor which sets initial parameters
    registration(float downSampleSize, float featureRadius, float maxIterationsSAC );

    //method for loading pointclouds efficiently enough
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointClouds(std::string path, std::string filetype);

    //method for downsampling via voxelization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float downSampleSize);

    //method for registration of point clouds
    Eigen::Matrix4f registerClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    //method to get fpfh features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeaturesFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                               pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                               double radius);

    //methods to determine normals in pointclouds
    pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);

private:

    //private member vars
    float downSampleSize;
    float featureRadius;
    float maxIterationsSAC;

};