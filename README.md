# carto_map_toolbox

## 简介
carto_map_toolbox是一个基于Cartographer的离线地图编辑工具，可以读取pbstream地图文件并通过ROS话题发布地图数据，在Rviz上可视化显示地图数据。与cartographer_ros中的`visualize_pbstream.launch`不同的是在carto_map中使用了独立的节点加载和显示地图数据，并且提供ROS service接口实现对地图的编辑，目前支持以下地图编辑功能：
- 子图数据删除
- 轨迹数据删除
- 手动回环优化
- 子图重叠率检测

参考：
https://github.com/sqrt81/carto_slam

## 安装编译
系统环境：ubuntu20.04、ROS noetic
> **注意:** 仓库内包含了原生的cartographer和cartographer_ros功能包，注意编译冲突

安装absl
安装glog


在本地创建ROS工作空间，将仓库克隆到工作空间然后编译
```
mkdir -p docker_ws/src
cd docker_ws/src
git clone <url>
cd ..
catkin_make_isolated
source devel_isolated/setup.bash
```


## 使用说明
使用前确认已经source工作空间的环境变量
```
source devel_isolated/setup.bash
```
使用roslaunch启动节点
```
roslaunch carto_map visualize_pbstream.launch pbstream_filename:=/home/dean/map/
```
其中变量`pbstream_filename`为加载地图文件夹路径
> **注意:** 输入参数为**地图文件夹路径**，不是pbstream地图文件的路径  

节点启动后会自动打开Rviz显示从文件中加载到的地图，然后可以通过调用ROS service接口实现对地图的编辑。

1. 删除子图数据
```
rosservice call /carto_map/remove_submap "trajectory_id: 0 submap_index: 1"
```

2. 删除轨迹数据
```
rosservice call /carto_map/remove_trajectory "trajectory_id: 1"
```

3. 保存地图
```
rosservice call /carto_map/save_map "filename: '/home/dean/map'"
```
- 参数`filename`为地图文件夹路径

4. 手动回环优化
```
rosservice call /carto_map/optimize_submap "{trajectory_id: 0, submap_index: 9, x: 0.0, y: 45.0, theta: 0.04}"
```

5. 子图重叠率检测
```
rosservice call /carto_map/compute_overlap_submap "{}"
```
此版本使用低分辨率子图进行重叠率检测，每次新生成的低分辨率子图会保存到meta.json文件中。
每次启动节点加载地图文件过程中，会从meta.json文件中读取之前生成好的低分辨率子图。



## TODO






