void 	glob (String pattern, std::vector< String > &result, bool recursive=false);

cv::FileStorage		read/write XML/YAML/JSON

Eigen::Quaternion< Scalar_, Options_ > //represent 3D orientations and rotations
Quaternion (const Scalar &w, const Scalar &x, const Scalar &y, const Scalar &z)	//四元组
QuaternionBase::toRotationMatrix	(	void 		)	const; //Convert the quaternion to a 3x3 rotation matrix
static void cv::eigen2cv	(	const Eigen::Tensor< _Tp, 3, _layout > & 	src,
OutputArray 	dst)	//Converts an Eigen::Tensor to a cv::Mat.	

template<typename PointT>
bool RgbLidarProcess<PointT>::ReadPointsCloud(std::string file_path,	PtCdtr<pcl::PointXYZ> raw_cloud_ptr)

int 	pcl::io::savePCDFileASCII (const std::string &file_name, const pcl::PointCloud< PointT > &cloud)

std::to_string()
std::stoi()

typedef boost::shared_ptr< const PointCloud<PointT> > pcl::PointCloud< PointT >::ConstPtr
typedef boost::shared_ptr<       PointCloud<PointT> > pcl::PointCloud< PointT >::Ptr
template<typename PointT> using PCLPtr = typename pcl::PointCloud<PointT>::Ptr;
pcl::PointCloud<PointXYZ>::Ptr    pclPtr( new pcl::PointCloud<pcl::PointXYZ> )
pcl::PointCloud<PointXYZRGB>::Ptr pclPtr( new pcl::PointCloud<pcl::PointXYZRGB> )

void cv::undistort	(	InputArray 	src,
	OutputArray 	dst,
	InputArray 	cameraMatrix,
	InputArray 	distCoeffs,
	InputArray 	newCameraMatrix = noArray() )		//镜头矫正
	
===========
Eigen!!!!
pcl!!!!!!
=====
2. mapping
---
老师的思路是传入PointXYZRGB, 在NdtMapping::NdtFrameMatch()中，以新形成的PointXYZ处理.
当要插入PointXYZ地图，颜色地图插入PointXYZRGB.

更好的方法是NDT底层直接处理PointXYZRGB

