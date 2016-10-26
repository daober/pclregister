#pragma once

#include <string.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>

#include <Eigen/Geometry>



class registration {

public:
    //constructor which sets initial parameters
    registration(float downSampleSize, float featureRadius, float maxIterationsSAC );

    //method for loading pointclouds efficiently enough
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointClouds(const std::string filename);

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

    //outlier removal for more accuracy in aligning of pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloud);


    //save aligned point cloud to disk
    int saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                     const std::string filename);


private:

    //private member vars
    float downSampleSize_;
    float featureRadius_;
    float maxIterationsSAC_;

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

};