#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include "filters.hpp"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filters::thresholdDepth(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float min_depth, float max_depth) {

    pcl::PassThrough<pcl::PointXYZRGB>::Ptr passThrough (new pcl::PassThrough<pcl::PointXYZRGB>());

    passThrough->setInputCloud(input);
    passThrough->setFilterFieldName("z");
    passThrough->setFilterLimits(min_depth, max_depth);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr threshold (new pcl::PointCloud<pcl::PointXYZRGB>());
    passThrough->filter(*threshold);

    return (threshold);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filters::voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float leaf_size) {

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud (input);
    voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel_grid.filter (*downsampled);

    return (downsampled);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filters::removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float radius, int min_neighbors) {

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
    radius_outlier_removal.setInputCloud (input);
    radius_outlier_removal.setRadiusSearch (radius);
    radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers (new pcl::PointCloud<pcl::PointXYZRGB>());
    radius_outlier_removal.filter (*inliers);

    return (inliers);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filters::applyFilters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float min_depth, float max_depth,
                          float leaf_size, float radius, float min_neighbors) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter (new pcl::PointCloud<pcl::PointXYZRGB>());

    filter = thresholdDepth(input, min_depth, max_depth);
    filter = voxelize(filter, leaf_size);
    filter = removeOutliers(filter, radius, min_neighbors);

    return (filter);
}