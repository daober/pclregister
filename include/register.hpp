#pragma once

#include <string.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>

#include <Eigen/Geometry>
#include <pcl/keypoints/sift_keypoint.h>


class registration {

public:

    registration();
    //constructor which sets initial parameters
    registration(float downSampleSize, float featureRadius, float maxIterationsSAC );

    //method for loading pointclouds efficiently enough
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointClouds(const std::string filename);

    //intial transform of point cloud
    int initRotation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename);

    //merge clouds based on transformation matrix
    Eigen::Matrix4f mergeClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, Eigen::Matrix4f &transform,  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgtfeat,
                                pcl::PointCloud<pcl::FPFHSignature33>::Ptr srcfeat);

    //method for downsampling via voxelization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float downSampleSize = 0.3);

    //method for registration of point clouds
    Eigen::Matrix4f registerClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, bool useFPFH = true, bool useICP = false);

    //method to get fpfh features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeaturesFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                                             pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                                             double radius);

    //determine sift keypoints
    pcl::PointCloud<pcl::PointWithScale>::Ptr getSIFTKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    //estimate correspondences
    pcl::Correspondences estimateCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt,
                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                                                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgtfeat,
                                                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr srcfeat);


    //methods to determine normals in pointclouds
    pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);

    //outlier removal for more accuracy in aligning of pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloud);


    //save aligned point cloud to disk
    int saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                     const std::string filename);

    int visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud);

private:

    //private member vars
    float downSampleSize_;
    float featureRadius_;
    float maxIterationsSAC_;

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

};