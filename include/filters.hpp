#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>



class Filters{

public:

    /** Use a PassThrough filter to remove points with depth values that are too large or too small */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    thresholdDepth (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                    float min_depth,
                    float max_depth);

/** Use a VoxelGrid filter to reduce the number of points */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
             float leaf_size);

/** Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    removeOutliers (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                    float radius,
                    int min_neighbors);

/** Apply a series of filters (threshold depth, downsample, and remove outliers) */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    applyFilters (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                  float min_depth,
                  float max_depth,
                  float leaf_size,
                  float radius,
                  float min_neighbors);


private:

};