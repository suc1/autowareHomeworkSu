#include <opencv2/opencv.hpp>

#include "camera.h"

using namespace std;

CCamera::CCamera(const string& name) : m_name(name) {

}

bool CCamera::ReadImage(const string& fileName, cv::Mat K_cam, cv::Mat D_cam) {
    m_image = cv::imread(fileName);
    if(m_image.empty()) return false;

    cv::Mat undistort_img; // = image.clone();
    cv::undistort(m_image, undistort_img, K_cam, D_cam);
    m_image = undistort_img;
    return true;
}