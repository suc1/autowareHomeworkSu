#include "lidarCamera.h"
#include "lidar.h"
#include "tf.h"
#include "camera.h"

namespace std {
	template <>
	class hash< cv::Point >{
	public :
		size_t operator()(const cv::Point &pixel_cloud ) const
		{
			return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
		}
	};
};

bool CLidarCamera::Fusion(int num) {
    CTF tf;
    //"~/autowareHomeworkSu/" fail
    if(!tf.Read("/home/cheng/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/config/sensors_calib_apollo.yml")) {
        return false;
    }

    CLidar lidar;
    if(!lidar.ReadPcd("/home/cheng/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/lidar/11673.pcd")) {
        return false;
    }

    CCamera front("T_vechicle_front_camera");
    if(!front.ReadImage("/home/cheng/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/front_camera/1571647370_656743.jpg",
        tf.K_cam_front_camera_, tf.D_cam_front_camera_)) {
        return false;
    }

    CCamera left("T_vechicle_left_back_camera");
    if(!left.ReadImage("/home/cheng/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/left_back_camera/1571647370_631318.jpg",
        tf.K_cam_left_back_camera_, tf.D_cam_left_back_camera_)) {
        return false;
    }

    CCamera right("T_vechicle_right_back_camera");
    if(!right.ReadImage("/home/cheng/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/apollo/right_back_camera/1571647370_631298.jpg",
        tf.K_cam_right_back_camera_, tf.D_cam_right_back_camera_)) {
        return false;
    }

    CloudFusionRGB(lidar.m_pclPtr[0], front.m_image, tf.T_front_camera_lidar_,      tf.K_cam_front_camera_,     tf.D_cam_front_camera_,      lidar.m_pclRGBPtr);
    CloudFusionRGB(lidar.m_pclPtr[1], left.m_image,  tf.T_left_back_camera_lidar_,  tf.K_cam_left_back_camera_, tf.D_cam_left_back_camera_,  lidar.m_pclRGBPtr);
    CloudFusionRGB(lidar.m_pclPtr[2], right.m_image, tf.T_right_back_camera_lidar_, tf.K_cam_right_back_camera_,tf.D_cam_right_back_camera_, lidar.m_pclRGBPtr);

    lidar.SavePcd("/home/cheng/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/data/saved_rgb_clouds/", 0);
    return true;
}

void CLidarCamera::CloudFusionRGB(pcl::PointCloud< pcl::PointXYZ >::Ptr in_cloud_ptr, 
                        cv::Mat undistort_img,
                        cv::Mat T_cam_lidar,
                        cv::Mat K_cam,
                        cv::Mat D_cam,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr)
{
    std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;
    
    int j = 0;

    for(size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ p_lidar(in_cloud_ptr->points[i].x, in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].z);
      pcl::PointXYZ p_cam;
      cv::Mat p_in = (cv::Mat_<double>(4,1) << p_lidar.x, p_lidar.y, p_lidar.z, 1);
      p_cam.x = T_cam_lidar.at<double>(0,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(0,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(0,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(0,3);
      p_cam.y = T_cam_lidar.at<double>(1,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(1,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(1,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(1,3);
      p_cam.z = T_cam_lidar.at<double>(2,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(2,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(2,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(2,3);

      int u = int(p_cam.x*K_cam.at<double>(0,0)/p_cam.z + K_cam.at<double>(0,2));
      int v = int(p_cam.y*K_cam.at<double>(1,1)/p_cam.z + K_cam.at<double>(1,2));
      if((u<=0) || (v<=0) || (u>=undistort_img.cols) || (v>=undistort_img.rows))
        continue;
      projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(u,v), p_lidar));
    }
    
    for (int row = 0; row < undistort_img.rows; row++)
    {
      for (int col = 0; col < undistort_img.cols; col++)
      {
        std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator iterator_3d_2d;
        pcl::PointXYZ corresponding_3d_point;
        pcl::PointXYZRGB colored_3d_point;
        iterator_3d_2d = projection_map.find(cv::Point(col, row));
        if (iterator_3d_2d != projection_map.end())
        {
          corresponding_3d_point = iterator_3d_2d->second;
          cv::Vec3b rgb_pixel = undistort_img.at<cv::Vec3b>(row, col);
          colored_3d_point.x = corresponding_3d_point.x;
          colored_3d_point.y = corresponding_3d_point.y;
          colored_3d_point.z = corresponding_3d_point.z;
          colored_3d_point.r = rgb_pixel[2];
          colored_3d_point.g = rgb_pixel[1];
          colored_3d_point.b = rgb_pixel[0];
          j++;
          out_cloud_ptr->points.push_back(colored_3d_point);
          //std::cout<< colored_3d_point.r <<","<< colored_3d_point.g<<","<< colored_3d_point.b<<std::endl;
        }
      }
    }

    out_cloud_ptr->height = 1;
	out_cloud_ptr->width += j;
}