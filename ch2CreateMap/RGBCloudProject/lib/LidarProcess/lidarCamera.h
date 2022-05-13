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
#include <opencv2/opencv.hpp>

class CLidarCamera {
    public:
        CLidarCamera() = default;
        ~CLidarCamera() = default;

        bool Fusion(int num);
    private:
        void CloudFusionRGB(pcl::PointCloud< pcl::PointXYZ >::Ptr in_cloud_ptr, 
                cv::Mat undistort_img,
                cv::Mat T_cam_lidar,
                cv::Mat K_cam,
                cv::Mat D_cam,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr);
};
