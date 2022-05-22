#include "gnss_projection_core.h"

Eigen::Matrix4d T_w2m;
cv::Mat T_w2m_cv;

GnssProjectionNode::GnssProjectionNode()
  : orientation_time_(-std::numeric_limits<double>::infinity()),
  position_time_(-std::numeric_limits<double>::infinity())
{
    initForROS();

    geo_.set_plane(plane_number_);
}

void GnssProjectionNode::initForROS()
{
    ros::NodeHandle private_nh_("~");
    // ros parameter settings
    private_nh_.getParam("plane", plane_number_);
    private_nh_.getParam("yaml_path", yaml_path_);
    private_nh_.getParam("pcd_path", pcd_path_);
    private_nh_.getParam("save_path", save_path_);

    // setup subscriber
    sub1_ = nh_.subscribe("nmea_sentence", 100, &GnssProjectionNode::callbackFromNmeaSentence, this);
}

void GnssProjectionNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
    if(m_alignMapDone) return;      //完成了两张新地图的对齐工作

    convert( split(msg->sentence) );

    //已经得到了姿态信息, 或者是gnss给的，或者是自己算出来的
    //这是新地图原点在世界坐标系的位置
    const double e = 1e-2;
    if ((fabs(orientation_time_ - position_time_) < e))
    {
        std::cout << "gnss working" <<std::endl;
        publishPoseStamped();   //m_alignMapDone = true;
        return;
    }
    
    //gnss 有可能没给姿态信息或者不同步，得自己计算
    static geo_pos_conv start_geo_ = geo_;
    double dt = sqrt(pow(geo_.x() - start_geo_.x(), 2) + pow(geo_.y() - start_geo_.y(), 2));
    const double threshold = 0.2;
    if (dt > threshold)
    {
        ROS_INFO("QQ is not received/Synchronized. Orientation is created by atan2");
        createOrientation(start_geo_);
        publishPoseStamped();   //m_alignMapDone = true;
    }
}

//copy autoware
std::vector<std::string> GnssProjectionNode::split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);
  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

void GnssProjectionNode::convert(std::vector<std::string> nmea)
{
  //std::cout << "gnss = " << msg->sentence << std::endl << std::endl;
  //gnss = QQ02C,INSATT,V,004035.40,7.226,14.376,223.220,@34
  //gnss = $GPGGA,004035.40,3514.1430181,N,13700.2620311,E,4,12,0.81,47.3559,M,38.4566,M,1.4,0556*46
  try
  {
    if (nmea.at(0).compare(0, 2, "QQ") == 0)
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
      ROS_INFO("QQ is subscribed.");
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      ROS_INFO("PASHR is subscribed.");
    }
    else if (nmea.at(0).compare(3, 3, "GGA") == 0)
    {
      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      double lon = stod(nmea.at(4));
      double h = stod(nmea.at(9));

      if (nmea.at(3) == "S")
        lat = -lat;

      if (nmea.at(5) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GGA is subscribed.");
    }
    else if (nmea.at(0) == "$GPRMC")
    {
      position_time_ = stoi(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;

      if (nmea.at(4) == "S")
        lat = -lat;

      if (nmea.at(6) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GPRMC is subscribed.");
    }
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

void GnssProjectionNode::createOrientation(const geo_pos_conv &last_geo_)
{
  yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  roll_ = 0;
  pitch_ = 0;
}

void GnssProjectionNode::publishPoseStamped()
{
  assert(!m_alignMapDone);

  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr_b(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr_b(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if(  ReadPointsCloud(pcd_path_ + "originB_csf.pcd", in_cloud_ptr_a) 
    && ReadPointsCloud(pcd_path_ + "originA_csf.pcd", in_cloud_ptr_b) 
    && ReadPointsCloud(pcd_path_ + "bin_Laser-00147_-00849.pcd", target_cloud_ptr))
  {
    //只考虑yaw，假设处于理想平面
    auto pose = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_);
    Eigen::Quaternionf q(pose.w, pose.x, pose.y, pose.z);

    Eigen::Matrix<float, 3, 3> R = q.toRotationMatrix();
    Eigen::Matrix<float, 3, 1> t;
    t<<geo_.y(), geo_.x(), geo_.z();

    Eigen::Matrix4f T_nm2m; //这里计算的是新建地图的原点所在坐标系到原地图坐标系下的坐标
    T_nm2m.setIdentity();
    T_nm2m.topLeftCorner<3,3>()=R;
    T_nm2m.topRightCorner<3,1>()=t;

    ReadExtrinsicsParam(yaml_path_);

    std::cout << T_nm2m << std::endl;
    //基于gnss换算后的坐标位置，将原始点云投影，可以理解为粗匹配
    TransformToMap(in_cloud_ptr_a, out_cloud_ptr_a, T_nm2m);
    TransformToMap(in_cloud_ptr_b, out_cloud_ptr_b, T_nm2m);

    pcl::io::savePCDFileASCII(save_path_ + "temp_result_a.pcd", *out_cloud_ptr_a);
    pcl::io::savePCDFileASCII(save_path_ + "temp_result_b.pcd", *out_cloud_ptr_b);

    //设定ndt初始值，凭感觉和经验，初始值很重要????  //Why not pose???
    Eigen::Matrix<float, 3, 1> t1;
    t1<<5, 5, -2.5;

    Eigen::Matrix4f T_init;
    T_init.setIdentity();
    T_init.topRightCorner<3,1>()=t1;
    
    Eigen::Matrix4f T_final = NdtMatching(out_cloud_ptr_a, target_cloud_ptr, T_init);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_a;
    cloud_transformed_a.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_b;
    cloud_transformed_b.reset(new pcl::PointCloud<pcl::PointXYZ>());

    //just set lidar and base coordinate coincide
    //基于ndt结果进行点云投影，可以理解为精匹配
    pcl::transformPointCloud(*out_cloud_ptr_a, *cloud_transformed_a, T_final);
    pcl::transformPointCloud(*out_cloud_ptr_b, *cloud_transformed_b, T_final);
    //最终的结果保存 可以直接用在官方demo中的地图加载
    pcl::io::savePCDFileASCII(save_path_ + "final_result_a.pcd", *cloud_transformed_a);
    pcl::io::savePCDFileASCII(save_path_ + "final_result_b.pcd", *cloud_transformed_b);

    m_alignMapDone = true;
  }  
}

bool GnssProjectionNode::ReadPointsCloud(std::string pcd_file, pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *in_cloud_ptr )== -1)
    {
        PCL_ERROR("Couldn't read file pcd file\n");
        return false;
    }
    
    return true;
}

void GnssProjectionNode::ReadExtrinsicsParam(std::string yaml_path)
{
  cv::FileStorage fsSettings(yaml_path, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cout << "File path is" <<yaml_path<< std::endl;
    std::cerr << "ERROR:Wrong path to settings" << std::endl;
    return;
  }

  fsSettings["world2map_ext"] >> T_w2m_cv;

  cv::cv2eigen(T_w2m_cv, T_w2m);
}

void GnssProjectionNode::run()
{
  while(!m_alignMapDone) {
    ros::spinOnce();
  }  
}


void GnssProjectionNode::TransformToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                                           Eigen::Matrix4f f_mat)
{
    out_cloud_ptr->points.clear();
    int j = 0;
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        pcl::PointXYZ p = in_cloud_ptr->points[i];
        cv::Mat p_in = (cv::Mat_<double>(4,1) << p.x, p.y, p.z, 1);
        
        double p_out_x =  f_mat(0,0)*p_in.at<double>(0,0)+f_mat(0,1)*p_in.at<double>(1,0)+f_mat(0,2)*p_in.at<double>(2,0)+f_mat(0,3);
        double p_out_y =  f_mat(1,0)*p_in.at<double>(0,0)+f_mat(1,1)*p_in.at<double>(1,0)+f_mat(1,2)*p_in.at<double>(2,0)+f_mat(1,3);
        double p_out_z =  f_mat(2,0)*p_in.at<double>(0,0)+f_mat(2,1)*p_in.at<double>(1,0)+f_mat(2,2)*p_in.at<double>(2,0)+f_mat(2,3);

        p.x = p_out_x;
        p.y = p_out_y;
        p.z = p_out_z;

        if(!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
        {
          j++;
          out_cloud_ptr->points.push_back(p);
        }
    }
    out_cloud_ptr->height = 1;
	  out_cloud_ptr->width = j;
}


Eigen::Matrix4f GnssProjectionNode::NdtMatching(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
                                                Eigen::Matrix4f init_mat)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_filtered;
    source_cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
    
    cpu_ndt_.setInputTarget(target_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid_filter.setInputCloud(source_cloud);
    voxel_grid_filter.filter(*source_cloud_filtered);
    
    const int max_iter_ = 30;        // Maximum iterations
    const float ndt_res_ = 1.0;      // Resolution
    const double step_size_ = 0.1;   // Step size
    const double trans_eps_ = 0.01;  // Transformation epsilon
    // set ndt source
    cpu_ndt_.setTransformationEpsilon(trans_eps_);
    cpu_ndt_.setStepSize(step_size_);
    cpu_ndt_.setResolution(ndt_res_);
    cpu_ndt_.setMaximumIterations(max_iter_);
    cpu_ndt_.setInputSource(source_cloud_filtered);  

    cpu_ndt_.align(init_mat);  

    std::cout<<"run here!"<<std::endl;
    double fitness_score = cpu_ndt_.getFitnessScore();
    Eigen::Matrix4f t_localizer = cpu_ndt_.getFinalTransformation();
    bool has_converged = cpu_ndt_.hasConverged();
    int final_num_iteration = cpu_ndt_.getFinalNumIteration();
    std::cout<<t_localizer<<std::endl;

    return t_localizer;
}

