#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <string.h>
#include "features.hpp"


/**
 * convenient helper class to load the point clouds from disk
 */

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
    
    int getSize();


private:
    
    int datacnt = 0;

};


/**
 * convenient helper class to save the point clouds to disk
 */


class Saver{

public:

    int saveObjectFeatures(std::string filename, boost::shared_ptr<Features::ObjectFeatures> &objFeatures);

    int savePoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points);

    int saveSurfaceNormals(std::string filename, pcl::PointCloud<pcl::Normal>::Ptr &normals);

    int saveKeypoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints);

    int saveLocalDescriptors(std::string filename, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &signature);

    int saveGlobalDescriptors(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr &signature);


private:


};
