#include <opencv2/opencv.hpp>

#include "camera.h"

using namespace std;

CCamera::CCamera(const string& name) : m_name(name) {

}

bool CCamera::ReadImage(const string& fileName) {
    m_image = cv::imread(fileName);
    return !m_image.empty();
}