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
    
    public:
        pcl::PointCloud< pcl::PointXYZ >::Ptr  m_pclPtr;
};
