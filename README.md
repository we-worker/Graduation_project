# 使用cartographer+navigation进行建图定位导航

![项目图片](./Doc/ranger_mode.png)

## 元件清单

- 底盘：ranger_mini-v2
- 激光雷达：pf的R2000
- IMU：taobotics的A9
- 主控：jeston nano 8GB
- 其他：明纬50V转24V电源、明纬50V转5V电源

## 技术栈

- 建图：使用cartographer（imu+laser）
- 定位：使用cartographer的纯定位（imu+laser）
- 导航：使用navigation+teb_local_planner

## 节点图

![node图片](./Doc/ros_node_graph.png)

![tf图片](./Doc/ros_tf_graph.png)

## 更新日志

- 初步大框架
- ranger_mini_urdf修正完成
- 修改imu为imu_link，机器人为base_link。成功部署carlike

## TODO
- movebase节点添加imu数据
- 修复全向移动时线速度和角速度同时有数据导致轮胎乱动的问题
- 使用MPC控制





## 安装步骤

安装ros1  使用鱼香ros一键安装  （jetson nano 使用ubuntu 18.04 melodic(ROS1)）
一键安装:rosdep(小鱼的rosdepc,又快又好用)
一键安装:VsCode开发工具
一键配置:python国内源
1. wget http://fishros.com/install -O fishros && . fishros

2.安装git
sudo apt-get install git

3.下载github仓库代码
git clone https://github.com/we-worker/Graduation_project.git
4.下载子git文件
cd Graduation_project
git submodule update --init  --recursive

4.编译IMU
cd Graduation_project/IMU
catkin_make

5.