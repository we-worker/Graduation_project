#激光雷达启动
gnome-terminal -t "pf_driver" -- bash -c "source /home/arc/works/Graduation_project/pf_laser/devel/setup.bash && roslaunch pf_driver r2000.launch"
sleep 3
#imu启动
gnome-terminal -t "imu" -- bash -c "source /home/arc/works/Graduation_project/IMU/devel/setup.sh && roslaunch handsfree_ros_imu handsfree_imu.launch"
sleep 3
#ranger底盘启动
# sudo modprobe gs_usb
# gnome-terminal -t "ranger" -- bash -c "source /home/arc/works/Graduation_project/ranger_ros/devel/setup.bash && roslaunch ranger_bringup ranger_mini_v2.launch"
# rosrun ranger_bringup bringup_can2usb.bash
echo 'arc' | sudo -S modprobe gs_usb && gnome-terminal -t "ranger" -- bash -c "source /home/arc/works/Graduation_project/ranger_ros/devel/setup.bash && rosrun ranger_bringup bringup_can2usb.bash && roslaunch ranger_bringup ranger_mini_v2.launch"
#ranger底盘启动键盘控制
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
sleep 3

gnome-terminal -t "carto" -- bash -c "source /home/arc/works/Graduation_project/cartographer/devel_isolated/setup.bash "


# gnome-terminal --tab -e 'bash -c "source /home/arc/works/Graduation_project/pf_laser/devel/setup.bash && roslaunch pf_driver r2000.launch; exec bash"' \
# --tab -e 'bash -c "roslaunch handsfree_ros_imu handsfree_imu.launch; exec bash"'

