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