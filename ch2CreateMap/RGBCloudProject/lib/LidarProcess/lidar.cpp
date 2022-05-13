#include <pcl/io/pcd_io.h>

#include "lidar.h"

using namespace std;
using namespace pcl;

#define _countof(array) (sizeof(array) / sizeof(array[0]))

CLidar::CLidar() {
    for(size_t i=0; i<_countof(m_pclPtr); ++i)
        m_pclPtr[i]= pcl::PointCloud< pcl::PointXYZ >::Ptr( new pcl::PointCloud<PointXYZ> );

    m_pclRGBPtr = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud<PointXYZRGB> );
    m_pclRGBPtr->height = 0;
	m_pclRGBPtr->width  = 0;
}

bool CLidar::ReadPcd(const string& fileName) {
    pcl::PointCloud< pcl::PointXYZ >::Ptr  allPtr( new pcl::PointCloud<PointXYZ> ); 
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *allPtr) == -1)
    {
        return false;
    }

    for( size_t i=0; i<allPtr->points.size(); ++i) {
        pcl::PointXYZ p = allPtr->points[i];
        if(p.x >= 0) m_pclPtr[0]->points.push_back(p);
        else if(p.y>0) m_pclPtr[1]->points.push_back(p);
        else m_pclPtr[2]->points.push_back(p);
    }
    return true;
}

void CLidar::SavePcd(const std::string& filePath, int num) {
    std::string saved_path = filePath + "/rgb_pcd_" + std::to_string(num) + ".pcd";
    pcl::io::savePCDFileASCII(saved_path, *m_pclRGBPtr);
}
