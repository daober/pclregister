#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>



class Filters{

public:

    /** Use a PassThrough filter to remove points with depth values that are too large or too small */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    thresholdDepth (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float min_depth, float max_depth);

/** Use a VoxelGrid filter to reduce the number of points */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float leaf_size);

/** Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    removeOutliers (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float radius, int min_neighbors);

/** Apply a series of filters (threshold depth, downsample, and remove outliers) */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    applyFilters (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float min_depth, float max_depth,
                  float leaf_size, float radius, float min_neighbors);

/** remove all NaN point cloud values */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    removeNaNPoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, const std::string filename);

/** remove all NaN values from Normals */
    pcl::PointCloud<pcl::PointNormal>::Ptr
    removeNaNNormals (const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormal, const std::string filename);

private:

};