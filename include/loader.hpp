#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <string.h>
#include "features.hpp"


class Loader{


public:
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
    loadPointCloud (std::string filename, std::string suffix);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    loadPoints (std::string filename);

    pcl::PointCloud<pcl::Normal>::Ptr
    loadSurfaceNormals(std::string filename);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    loadKeypoints (std::string filename);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
    loadLocalDescriptors (std::string filename);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr
    loadGlobalDescriptors (std::string filename);

    int saveObjectFeatures(boost::shared_ptr<Features::ObjectFeatures> &input);


private:

};