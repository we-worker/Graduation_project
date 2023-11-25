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
- 上位机：QT+socket

## 节点图

![node图片](./Doc/ros_node_graph.png)

![tf图片](./Doc/ros_tf_graph.png)

## 上位机

![上位机图片](./QT-host-controller/Doc/效果展示.png)

## 更新日志

- 初步大框架
- ranger_mini_urdf修正完成
- 修改imu为imu_link，机器人为base_link。成功部署carlike
- 搭建上位机大框架

## TODO
- movebase节点添加imu数据
- 修复全向移动时线速度和角速度同时有数据导致轮胎乱动的问题
- 使用MPC控制





## 安装步骤


1. **下载并安装鱼香ros**
   - 安装ros1：使用鱼香ros一键安装（jetson nano 使用ubuntu 18.04 melodic(ROS1)）
   - 一键安装：rosdep(小鱼的rosdepc,又快又好用)
   - 一键安装：VsCode开发工具
   - 一键配置：python国内源
    ```bash
    wget http://fishros.com/install -O fishros && . fishros
    ```

2. **安装git**
    ```bash
    sudo apt-get install git
    ```

3. **下载github仓库代码**
    ```bash
    git clone https://github.com/we-worker/Graduation_project.git
    ```

4. **下载子git仓库文件**，如果这一步迟迟未响应，修改.gitmodules文件中的github链接为镜像连接。
    ```bash
    cd Graduation_project
    git submodule update --init  --recursive
    ```

5. **编译IMU**
    ```bash
    cd Graduation_project/IMU
    catkin_make
    ```

6. **编译pf激光雷达**
    ```bash
    cd Graduation_project/pf_laser
    rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
    catkin_make
    ```

7. **编译ranger底盘ros驱动器**
    ```bash
    sudo apt install libasio-dev libboost-all-dev
    cd Graduation_project/ranger_ros
    catkin_make
    ```

8. **编译navigation**
    ```bash
    cd Graduation_project/navigation
    catkin_make
    ```

9.  **编译cartographer**
    ```bash
    cd Graduation_project/cartographer
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    sudo apt-get install -y clang cmake g++ google-mock \
        libboost-all-dev \
        libcairo2-dev \
        libcurl4-openssl-dev \
        libeigen3-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        liblua5.2-dev \
        libsuitesparse-dev \
        lsb-release \
        ninja-build \
        stow
    sudo src/cartographer/scripts/install_abseil.sh
    解压meshes材质文件
    tar -xJvf ./src/mycarto/meshes/meshes.tar.xz -C ./src/mycarto/meshes/
    catkin_make_isolated --install --use-ninja
    ```

10. **安装一个分屏终端方便使用**
    ```bash
    sudo apt-get install terminator
    ```

11. **启动方法**见Doc目录，start.sh文件
