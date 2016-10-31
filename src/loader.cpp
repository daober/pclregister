#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include "loader.hpp"


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
Loader::loadPointCloud(std::string filename, std::string suffix) {

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > output (new pcl::PointCloud<pcl::PointXYZRGB>);
    filename.append (suffix);

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Loader::loadPoints(std::string filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>());
    filename.append ("_points.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr Loader::loadLocalDescriptors(std::string filename) {
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr output (new pcl::PointCloud<pcl::FPFHSignature33>());
    filename.append ("_localdesc.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr Loader::loadGlobalDescriptors(std::string filename) {
    pcl::PointCloud<pcl::VFHSignature308>::Ptr output (new pcl::PointCloud<pcl::VFHSignature308>());
    filename.append ("_globaldesc.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}

pcl::PointCloud<pcl::Normal>::Ptr Loader::loadSurfaceNormals(std::string filename) {
    pcl::PointCloud<pcl::Normal>::Ptr output (new pcl::PointCloud<pcl::Normal>());
    filename.append ("_normals.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Loader::loadKeypoints(std::string filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>());
    filename.append ("_keypoints.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s (%zu points)\n", filename.c_str (), output->size ());
    return (output);
}