# Ubuntu 20.04 Cartographer ROS 安装使用指南

参考链接：[CSDN博客](https://blog.csdn.net/m0_46482248/article/details/126745758)

## Cartographer 编译安装

```bash
catkin_make_isolated --install --use-ninja
source devel_isolated/setup.bash
```

## 建图

1. 结束建图，先停止建图：

```bash
rosservice call /finish_trajectory 0
```

2. 然后保存成pbstream，自己更改保存路径，注意不能写相对路径，测试过不行：

```bash
rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"
```

3. 导出成一般导航使用的yaml+pgm：

```bash
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05
```

## 录制包

```bash
rosbag record -a
```

## 建图方式

- 纯激光建图：

```bash
roslaunch cartographer_ros demo_revo_lds.launch
```

- 激光+imu建图：

```bash
roslaunch cartographer_ros dyp_backpack_2d.launch 
```

## 报错原因总结：

1. 是否使用了imu数据，lua文件
2. 节点是否重映射，imu，scan
3. urdf文件中，imu，lidar绑定的fram id 也就是关节是否对应
4. `<param name="/use_sim_time" value="false" />` 设置为true，此时说明系统使用的是仿真时间，如果设置为false，则系统使用walltime

## 2D数据测试

### 纯定位仿真测试

官方文档提供了纯定位的仿真教程，步骤如下：

1. 下载数据集：

```bash
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-05-14-44-52.bag
```

2. 建图：

```bash
roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
```

3. 纯定位测试：

这里后面两个参数一个是上一步建好的地图，pbstream格式，另一个是数据集。也就是说，先用第一个数据集建图，建完图后用第二个数据集进行定位测试。

```bash
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag  

roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=/home/arc/works/Graduation_project/cartographer/cartographer_paper_deutsches_museum.bag
```

