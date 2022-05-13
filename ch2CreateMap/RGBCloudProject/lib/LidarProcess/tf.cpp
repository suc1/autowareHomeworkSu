#include "tf.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/eigen.hpp>

bool CTF::Read(const std::string & fileName) {
    cv::FileStorage fsSettings(fileName, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR:Wrong path for settings: " << fileName << std::endl;
        return false;
    }

    T_vechicle_lidar_= ReadTf4x4(fsSettings, "T_vechicle_lidar");

    ReadCameraTf( fsSettings, "T_vechicle_front_camera",      "K_front_camera",      "D_front_camera",
        T_front_camera_lidar_,      K_cam_front_camera_,      D_cam_front_camera_);
    
    ReadCameraTf( fsSettings, "T_vechicle_left_back_camera",  "K_left_back_camera",  "D_left_back_camera",
        T_left_back_camera_lidar_,  K_cam_left_back_camera_,  D_cam_left_back_camera_);

    ReadCameraTf( fsSettings, "T_vechicle_right_back_camera", "K_right_back_camera", "D_right_back_camera",
        T_right_back_camera_lidar_, K_cam_right_back_camera_, D_cam_right_back_camera_);
    return true;
}

cv::Mat CTF::ReadTf4x4( cv::FileStorage &fs, const std::string &section ) 
{
    cv::Mat T;
    fs[section] >> T;

    if(T.cols == 4 && T.rows == 4)
    {
        return T;
    }
    Eigen::Quaterniond q(T.at<double>(3,0), T.at<double>(0,0), T.at<double>(1,0), T.at<double>(2,0));
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix();
    Eigen::Matrix<double, 3, 1> t;
    t<<T.at<double>(4,0), T.at<double>(5,0), T.at<double>(6,0);

    Eigen::Matrix4d T_temp;
    T_temp.setIdentity();
    T_temp.topLeftCorner<3,3>()  = R;
    T_temp.topRightCorner<3,1>() = t;

    cv::Mat T_final;
    cv::eigen2cv(T_temp, T_final);
    return T_final;
}

//ToDo: 弄明白座标变换
void CTF::ReadCameraTf( cv::FileStorage &fsSettings, const std::string &section, const std::string &kCam, const std::string &dCam,
    cv::Mat& T_front_camera_lidar_, cv::Mat& K_cam_front_camera_, cv::Mat& D_cam_front_camera_) 
{
    cv::Mat T_vechicle_front_camera_ = ReadTf4x4(fsSettings, section);
    fsSettings[kCam] >> K_cam_front_camera_;
    fsSettings[dCam] >> D_cam_front_camera_;

    cv::Mat T_front_camera_vechicle;
    cv::invert(T_vechicle_front_camera_, T_front_camera_vechicle);
    T_front_camera_lidar_ =  T_front_camera_vechicle * T_vechicle_lidar_;
}
