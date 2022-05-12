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

class CTF {
    public:
        CTF() = default;
        ~CTF() = default;

        bool Read(const std::string & fileName);
    private:
        cv::Mat ReadTf4x4( cv::FileStorage &fs, const std::string &section );
    public:
        cv::Mat m_lidar2CameraFront;
        cv::Mat m_lidar2CameraLeft;
        cv::Mat m_lidar2CameraRight;
};