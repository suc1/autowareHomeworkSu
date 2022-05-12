#include <pcl/io/pcd_io.h>

#include "lidar.h"

using namespace std;
using namespace pcl;

CLidar::CLidar() : m_pclPtr(new pcl::PointCloud<PointXYZ>) {

}

bool CLidar::ReadPcd(const string& fileName) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *m_pclPtr) == -1)
    {
        return false;
    }

    return true;
}