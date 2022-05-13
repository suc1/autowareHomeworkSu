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
        void ReadCameraTf( cv::FileStorage &fsSettings, const std::string &section, const std::string &kCam, const std::string &dCam,
                cv::Mat& T_front_camera_lidar_, cv::Mat& K_cam_front_camera_, cv::Mat& D_cam_front_camera_);
    public:
        //ToDo: Perfect???
        cv::Mat m_lidar2CameraFront;
        cv::Mat m_lidar2CameraLeft;
        cv::Mat m_lidar2CameraRight;
    public:
        //ToDo: 应该上面三个座标就可以了
        cv::Mat T_vechicle_lidar_; //lidar to vechicle

        cv::Mat T_front_camera_lidar_;
        cv::Mat K_cam_front_camera_;
        cv::Mat D_cam_front_camera_;

        cv::Mat T_left_back_camera_lidar_;
        cv::Mat K_cam_left_back_camera_;
        cv::Mat D_cam_left_back_camera_;

        cv::Mat T_right_back_camera_lidar_;
        cv::Mat K_cam_right_back_camera_;
        cv::Mat D_cam_right_back_camera_;
};