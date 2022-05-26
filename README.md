# autowareHomeworkSu
My homework for autoware.ai

## Ch2-1: 给点云加上颜色
1. Build
```
cd ~/autowareHomeworkSu/ch2CreateMap/RGBCloudProject
cmake .
make
```
2. 效果图: ~/autowareHomeworkSu/ch2CreateMap/RGBCloudProject/ch2Result.png
3. 存在问题1: 左右点云有缺失，怀疑CLidar::ReadPcd()分配相机有Bugs. 老师的代码有同样问题
4. 存在问题2: CLidar::SavePcd()有Bugs, 长度有异
5. 存在问题3: CTF三个座标是完美方案???

## Ch2-2: 生成点云地图
1. Build
```
cd ~/autowareHomeworkSu/ch2CreateMap/Mapping
cmake .
make
```
2. 效果图: ~/autowareHomeworkSu/ch2CreateMap/Mapping/ch2Result-map.png
3. 存在问题1: 老师输出9个子图，而我只输出1个子图，原因是Lidar因为靠近被丢弃???

## Ch3-Gnss align addition map
```
#要先编译 "gnss" & "ndt_cpu", 最后编译 "gnss_projection"
# Move "gnss_projection" to another folder
cd ch3Localize
catkin_make -DCATKIN_WHITELIST_PACKAGES="gnss_projection"

# Restore "gnss_projection" to ~
catkin_make -DCATKIN_WHITELIST_PACKAGES="gnss_projection"

source ./devel/setup.bash
roslaunch  gnss_projection  gnss_projection.launch

cd ~/autoware.ai/relative_files
rosbag play sample_moriyama_150324.bag
```
1. 存在问题: 工程文件的依赖问题，CMake???
2. 存在问题: 一开始点云比老师的稠密
3. 存在问题: 看不出粗匹配和精匹配差异
4. 存在问题: 一开始点云基本对，但是经过Rebase，出现偏移!!!!

## Ch4 项目练习3: 基于欧式聚类的车辆分割
```
//滤波、切割、去地面、聚类、可视化的顺序来进行
//入口函数:
void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)

//去掉邻近的点云
void removePointsUpTo()

//降采样
void downsampleCloud()

//高度裁减
void clipCloud()

//仅仅关注lane范围内的点云，相当于对点云左右进行裁减
void keepLanePoints()

//移除地面
void removeFloor()

//采用差分法线特征算法再对点云进行一次过滤
void differenceNormalsSegmentation()

//正式进入欧式聚类的接口
void segmentByDistance()

```
