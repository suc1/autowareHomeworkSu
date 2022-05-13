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

#include <opencv2/opencv.hpp>

class CCamera {
    public:
        CCamera(const std::string& name);
        ~CCamera() = default;

        bool ReadImage(const std::string& fileName, cv::Mat K_cam, cv::Mat D_cam);
    
    public:
        std::string  m_name;
        cv::Mat m_image;
};