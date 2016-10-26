#include "register.hpp"



registration::registration(float downSampleSize, float featureRadius, float maxIterationsSAC) {



}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::loadPointClouds(std::string path, std::string filetype) {



}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr registration::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                              float downSampleSize) {


}

Eigen::Matrix4f registration::registerClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {


}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr registration::getFeaturesFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                                         double radius) {



}

pcl::PointCloud<pcl::Normal>::Ptr registration::getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud) {



}