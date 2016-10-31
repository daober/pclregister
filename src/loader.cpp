#include "loader.hpp"

#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
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


/***************************************************************************************************************/
/************************************************SEPERATOR******************************************************/
/***************************************************************************************************************/


int Saver::saveGlobalDescriptors(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr &signature) {
    int err = 0;
    filename.append ("_globaldesc.pcd");

    err = pcl::io::savePCDFile(filename, *signature);

    return (err);
}


int Saver::saveLocalDescriptors(std::string filename, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &signature) {
    int err = 0;
    filename.append ("_localdesc.pcd");

    err = pcl::io::savePCDFile(filename, *signature);

    return (err);
}


int Saver::saveKeypoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints) {
    int err = 0;
    filename.append ("_keypoints.pcd");

    err = pcl::io::savePCDFile(filename, *keypoints);

    return (err);
}


int Saver::saveSurfaceNormals(std::string filename, pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    int err = 0;
    filename.append ("_normals.pcd");

    err = pcl::io::savePCDFile(filename, *normals);

    return (err);
}


int Saver::savePoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points) {
    int err = 0;
    filename.append ("_points.pcd");

    err = pcl::io::savePCDFile(filename, *points);

    return (err);
}

int Saver::saveObjectFeatures(std::string filename, boost::shared_ptr<Features::ObjectFeatures> &objFeatures) {
    int err = 0;

    return (err);
}


