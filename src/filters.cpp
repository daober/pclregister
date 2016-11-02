#include "filters.hpp"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/filter.h>

#include <iostream>
#include <limits>




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



pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filters::removeNaNPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, const std::string filename) {

    pcl::console::print_highlight ("removing NaN values from cloud...\n");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PointCloud<pcl::PointXYZRGB>::PointType p_nan;
    pcl::PointCloud<pcl::PointXYZRGB>::PointType p_valid;

    p_nan.x = std::numeric_limits<float>::quiet_NaN();
    p_nan.y = std::numeric_limits<float>::quiet_NaN();
    p_nan.z = std::numeric_limits<float>::quiet_NaN();

    filteredCloud->push_back(p_nan);

    p_valid.x = 1.0f;
    filteredCloud->push_back(p_valid);

    std::cout<<"previous size of "<< filename <<" is: " << inputCloud->points.size() << std::endl;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*inputCloud, *filteredCloud, indices);

    std::cout<<"filtered size of "<< filename <<" is: " << filteredCloud->points.size() << std::endl;

    return filteredCloud;
}



pcl::PointCloud<pcl::PointNormal>::Ptr
Filters::removeNaNNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormal, const std::string filename) {

    pcl::console::print_highlight ("removing NaN values from normals...\n");

    pcl::PointCloud<pcl::PointNormal>::Ptr filteredNormal (new pcl::PointCloud<pcl::PointNormal>());

    pcl::PointCloud<pcl::PointNormal>::PointType p_nan;
    pcl::PointCloud<pcl::PointNormal>::PointType p_valid;

    p_nan.x = std::numeric_limits<float>::quiet_NaN();
    p_nan.y = std::numeric_limits<float>::quiet_NaN();
    p_nan.z = std::numeric_limits<float>::quiet_NaN();

    filteredNormal->push_back(p_nan);

    p_valid.x = 1.0f;
    filteredNormal->push_back(p_valid);

    std::cout<<"previous size of "<< filename <<" is: " << inputNormal->points.size() << std::endl;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*inputNormal, *filteredNormal, indices);

    std::cout<<"filtered size of "<< filename <<" is: " << filteredNormal->points.size() << std::endl;

    return filteredNormal;
}

