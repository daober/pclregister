#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>



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



private:

};