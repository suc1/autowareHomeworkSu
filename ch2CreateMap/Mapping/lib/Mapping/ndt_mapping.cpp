//#include <tf/transform_datatypes.h>
#include "ndt_mapping.h"


void NdtMapping::CreateNdtRgbMap(std::vector<cv::String> lidar_pathes)
{
    ReadLidarFrames(lidar_pathes);
    CreateMapBasedLidarFrames(mapping_frames_);
}

void NdtMapping::ReadLidarFrames(std::vector<cv::String> lidar_pathes)
{
    std::vector<LidarFrame> lidar_frames;

    for(size_t i = 0; i < lidar_pathes.size(); i++)
    {
        if(i%10==0) std::cout << "\r\nReadLidarFrames = " << i << std::endl;

        LidarFrame lidar_data;
        lidar_data.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        if (LoadPCDFile(lidar_pathes[i], *lidar_data.cloud) == -1) {
          std::cerr << "cannot open " << lidar_pathes[i] << std::endl;
          return;
        }

        std::string lidar_path = lidar_pathes[i];
        lidar_data.time_stamp = lidar_path.substr(lidar_path.length()-23,lidar_path.length());//TODO config
        double t_ms = atof(lidar_data.time_stamp.substr(10, 4).c_str())/10000;
        double t_s = atof(lidar_data.time_stamp.substr(6, 4).c_str());
        lidar_data.time_s =  t_s+t_ms;

        if(!NdtFrameMatch(lidar_data.cloud, lidar_data.time_s))
          std::cout << "ReadLidarFrames::NdtFrameMatch:discardFrame " << i << std::endl;
    }
    std::cout << "ReadLidarFrames = " << lidar_frames.size() << std::endl;
    std::cout << "ReadLidarFrames:mapping_frames_ = " << mapping_frames_.size() << std::endl;
}

//static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
bool NdtMapping::NdtFrameMatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp, double current_scan_time)
{
  double r;
  pcl::PointXYZ p;
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());

  //note-tianyu 点云过滤
  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator item = tmp->points.begin(); item != tmp->points.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range)
    {
      scan.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));

  // Add initial point cloud to velodyne_map
  // note-tianyu 第一帧scan输入，会将其当成直接插入到map中，作为组成map的第一帧子图
  static int initial_scan_loaded = 0;
  if (initial_scan_loaded == 0)
  {
    map += *scan_ptr;
    initial_scan_loaded = 1;
  }

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  //note-tianyu 根据对ndt优化的相关参数进行配置
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

  /*NDT算法原理：
　　NDT 算法的基本思想是，先对待配准点云进行栅格化处理，将其划分为指定大小的网格，通过正态分布的方 式，构建每个网格的概率分布函数，
    之后优化求解出最优变换参数，使得源点云概率密度分布达到最大，以实现两个点云之间的最佳匹配*/
  anh_ndt.setTransformationEpsilon(trans_eps);  //收敛数
  anh_ndt.setStepSize(step_size);               //步长
  anh_ndt.setResolution(ndt_res);               //格子边长
  anh_ndt.setMaximumIterations(max_iter);       //迭代次数
  anh_ndt.setInputSource(filtered_scan_ptr);
  

  static bool is_first_map = true;
  if (is_first_map == true)
  {
    anh_ndt.setInputTarget(map_ptr);
    is_first_map = false;
  }

  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  pose guess_pose_for_ndt;
  guess_pose_for_ndt = guess_pose;

  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);
  //note-tianyu ndt优化的初始值预测
  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  
  anh_ndt.align(init_guess);
  //fitness_score = anh_ndt.getFitnessScore();
  t_localizer = anh_ndt.getFinalTransformation();
  //has_converged = anh_ndt.hasConverged();
  //final_num_iteration = anh_ndt.getFinalNumIteration();
  
  t_base_link = t_localizer;//note-tianyu 坐标转换
  //note-tianyu 根据ndt匹配优化结果将scan的坐标由lidar坐标系换算到map坐标系下
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  // Update ndt_pose.
  // note-tianyu ndt_pose为地图坐标系到车身坐标系的位姿
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  
  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;

  // Calculate the offset (curren_pos - previous_pos)
  // note-tianyu 计算前后两帧pose的diff量
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  // Calculate the shift between added_pos and current_pos
  //note-tianyu 计算两frame之间的距离间隔，当大于一定阈值时，才像向地图中插入新的点云（子地图）
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift)
  {
    map += *transformed_scan_ptr;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;

    anh_ndt.setInputTarget(map_ptr);

    LidarFrame mapping_frame;
    mapping_frame.time_s = current_scan_time;
    mapping_frame.T_map_lidar = t_localizer;
    mapping_frame.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    size_t discardPt =0;
    for(size_t j = 0; j < tmp->points.size(); j++)
    {
      pcl::PointXYZRGB p_rgb;
      p_rgb = tmp->points[j];
      if(std::hypot(p_rgb.x, p_rgb.y) > min_scan_range && std::hypot(p_rgb.x, p_rgb.y) < max_scan_range)
        mapping_frame.cloud->points.push_back(p_rgb);
      else ++discardPt;
    }

    mapping_frames_.push_back(mapping_frame);

    if(discardPt != tmp->points.size()) {
       std::cout << "NdtFrameMatch:discardPt = " << discardPt << ", sum = " << tmp->points.size() << std::endl;
    }
    return true;
  }
  return false;
}

NdtMapping::NdtMapping()
{
    map_filtered_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_rgb_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    map_rgb_filtered_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    previous_pose.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    diff_pose.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    guess_pose.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    added_pose.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    current_pose.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}


double NdtMapping::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}

void NdtMapping::CreateMapBasedLidarFrames(std::vector<LidarFrame> &lidar_frames)
{
    if(lidar_frames.empty() || lidar_frames[0].T_map_lidar.rows() != 4)
    {
        std::cout<<"lidar_frames has no data!"<<std::endl;
        return;
    }

    int map_dispart_count = int(lidar_frames.size()/10 + 1);
    std::cout<<"map_dispart_count is: "<<map_dispart_count<<std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_map;
    sub_map.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    int sub_map_width = 0;
    int submap_id = 0;

    for(size_t i = 0; i < lidar_frames.size(); i++)
    {
        int num = i+1;

        if((num%int(10)) == 0)
        {
            sub_map->width = sub_map_width;
            sub_map->height = 1;
            std::string saved_path = "submap_" + std::to_string(submap_id) + ".pcd";
            std::cout<<submap_id<<"th submap saved success!"<<std::endl;

            pcl::io::savePCDFileASCII(saved_path, *sub_map);

            sub_map ->points.clear();
            sub_map_width = 0;

            submap_id++;
        }

        //std::cout<<lidar_frames[i].cloud->points.size()<<std::endl;

        for(size_t j = 0; j<lidar_frames[i].cloud->points.size(); j++)
        {
            pcl::PointXYZRGB lidar_p = lidar_frames[i].cloud->points[j];
            //Eigen::Matrix4f T =  lidar_frames[i].T_map_lidar;
            // std::cout<<"T_read: "<<T<<std::endl;

            double map_p_x =  lidar_frames[i].T_map_lidar(0,0)*lidar_p.x+lidar_frames[i].T_map_lidar(0,1)*lidar_p.y+lidar_frames[i].T_map_lidar(0,2)*lidar_p.z+lidar_frames[i].T_map_lidar(0,3);
            double map_p_y =  lidar_frames[i].T_map_lidar(1,0)*lidar_p.x+lidar_frames[i].T_map_lidar(1,1)*lidar_p.y+lidar_frames[i].T_map_lidar(1,2)*lidar_p.z+lidar_frames[i].T_map_lidar(1,3);
            double map_p_z =  lidar_frames[i].T_map_lidar(2,0)*lidar_p.x+lidar_frames[i].T_map_lidar(2,1)*lidar_p.y+lidar_frames[i].T_map_lidar(2,2)*lidar_p.z+lidar_frames[i].T_map_lidar(2,3);

            //std::cout<<"T: "<<lidar_frames[i].T_map_lidar<<std::endl;
            //std::cout<<"lidar: "<<lidar_p.x<<","<<lidar_p.y<<","<<lidar_p.z<<std::endl;
            //std::cout<<"map: "<<map_p_x<<","<<map_p_y<<","<<map_p_z<<std::endl;

            lidar_p.x = map_p_x;
            lidar_p.y = map_p_y;
            lidar_p.z = map_p_z;

            sub_map->points.push_back(lidar_p);
            sub_map_width++;
        }
    }
    return;
}

int  NdtMapping::LoadPCDFile (const std::string &file_name, pcl::PointCloud<pcl::PointXYZRGB> &lidar_data) {
  //[pcl::PCDReader::readHeader] No points to read
  /*if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (lidar_pathes[i], *lidar_data.cloud) == -1) {
      std::cerr << "cannot open " << lidar_pathes[i] << std::endl;
      return;
    } */
  std::ifstream lidarfile(file_name);
  if (!lidarfile.is_open())
  {
    std::cerr << "lidar file cannot openned !" << std::endl;
    return -1;
  }

  std::string lidarline;
  while (std::getline(lidarfile, lidarline))
  {
    std::stringstream ss(lidarline);
    std::string str;
    pcl::PointXYZRGB p;

    ss >> str; p.x = atof(str.c_str());
    ss >> str; p.y = atof(str.c_str());
    ss >> str; p.z = atof(str.c_str());
    ss >> str; p.r = uint8_t(atoi(str.c_str()));
    ss >> str; p.g = uint8_t(atoi(str.c_str()));
    ss >> str; p.b = uint8_t(atoi(str.c_str()));
    lidar_data.points.push_back(p);
  }

  return lidar_data.points.size();
}
