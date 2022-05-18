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

