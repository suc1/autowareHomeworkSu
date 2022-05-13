/*********************************************************************
*
*  Copyright (c) 2022, SoftGold
*  All rights reserved.
*
*  
*  
*  
*  
*   Init: Cheng Su, May 11 2022
*********************************************************************/
#include <iostream>

#include <pcl/common/common.h>

class CLidar {
    public:
        CLidar();
        ~CLidar() = default;

        bool ReadPcd(const std::string& fileName);
        void SavePcd(const std::string& filePath, size_t num);
    public:
        pcl::PointCloud< pcl::PointXYZ >::Ptr     m_pclPtr[3]; //front, left, right
        pcl::PointCloud< pcl::PointXYZRGB >::Ptr  m_pclRGBPtr;
};
