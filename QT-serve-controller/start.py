import subprocess
import threading
import signal
import time

# 定义你的命令
# 1.启动激光雷达
# 2.启动IMU
# 3.启动ranger底盘
# 4.启动cartographer纯定位
# 5.启动导航控制
commands = {
    '1': 'source /home/arc/works/Graduation_project/pf_laser/devel/setup.bash && roslaunch pf_driver r2000.launch',
    '2': 'sudo chmod 777 /dev/ttyUSB0 && source /home/arc/works/Graduation_project/IMU/devel/setup.sh && roslaunch handsfree_ros_imu handsfree_imu.launch',
    '3': 'sudo -S modprobe gs_usb && source /home/arc/works/Graduation_project/ranger_ros/devel/setup.bash && rosrun ranger_bringup bringup_can2usb.bash && roslaunch ranger_bringup ranger_mini_v2.launch',
    '4': 'cd /home/arc/works/Graduation_project && source /home/arc/works/Graduation_project/cartographer/devel_isolated/setup.bash && roslaunch mycarto dyp_backpack_2d_localization.launch  load_state_filename:=./Map_bag_database/902/902.pbstream',
    '5': 'source /home/arc/works/Graduation_project/navigation/devel/setup.bash && roslaunch ranger_2dnav dyp_carlike.launch'
}

# 存储运行的进程
processes = {}

def run_command(cmd_id,client_socket=None):
    # 使用shell=True运行命令
    process = subprocess.Popen(commands[cmd_id], shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # 存储进程
    processes[cmd_id] = process

    # 打印输出
    while True:
        output = process.stdout.readline()
        if output == b'' and process.poll() is not None:
            break
        if output:
            print(f'{cmd_id}: {output.decode().strip()}')
    process.poll()
    if client_socket!=None:
        client_socket.sendall(f'OK START {cmd_id}'.encode())

def stop_command(cmd_id,client_socket=None):
    if cmd_id in processes:
        # 发送SIGTERM信号
        processes[cmd_id].send_signal(signal.SIGTERM)
        if client_socket!=None:
            client_socket.sendall(f'OK STOP {cmd_id}'.encode())


# 运行命令
# for cmd_id in ['1', '2', '3', '4', '5']:
#     thread = threading.Thread(target=run_command, args=(cmd_id,))
#     thread.start()
# thread = threading.Thread(target=run_command, args=('2',))
# thread.start()
# # 停止命令
# time.sleep(10)
# stop_command('1')