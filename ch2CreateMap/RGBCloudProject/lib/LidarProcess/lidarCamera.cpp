#include "lidarCamera.h"
#include "lidar.h"
#include "tf.h"
#include "camera.h"

bool CLidarCamera::Fusion(int num) {
    CTF tf;
    if(!tf.Read("~/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/config/sensors_calib_apollo.yml")) {
        return false;
    }

    CLidar lidar;
    if(!lidar.ReadPcd("~/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/lidar/11673.pcd")) {
        return false;
    }

    CCamera front("T_vechicle_front_camera");
    if(!front.ReadImage("~/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/front_camera/1571647370_656743.jpg")) {
        return false;
    }

    CCamera left("T_vechicle_left_back_camera");
    if(!left.ReadImage("~/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/left_back_camera/1571647370_631318.jpg")) {
        return false;
    }

    CCamera right("T_vechicle_right_back_camera");
    if(!right.ReadImage("~/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/right_back_camera/1571647370_631298.jpg")) {
        return false;
    }

    return true;
}