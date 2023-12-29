#!/usr/bin/python
# -*- coding: UTF-8 -*-

from actionlib.action_client import GoalManager
import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped


import cv2
import numpy as np
from matplotlib import pyplot as plt

import json

# 读取配置文件
with open('navigation/src/ranger_2dnav/scripts/config.json') as f:
    config = json.load(f)

pixel_to_meter_ratio = config['pixel_to_meter_ratio']
distance_in_pixels = config['distance_in_meters'] / pixel_to_meter_ratio
Parking_weight = config['Parking_weight'] / pixel_to_meter_ratio
Parking_height = config['Parking_height'] / pixel_to_meter_ratio



class ImageProcessor:
    def __init__(self, img_path):
        # 读取图像并转换为灰度图
        self.img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    def find_first_black_pixel(self, start, angle):
        # 从给定的起点和角度开始，找到第一个黑色像素
        x, y = start
        while 0 <= x < self.img.shape[1] and 0 <= y < self.img.shape[0]:
            if self.img[int(y), int(x)] < 128:
                return (x, y)
            x += math.sin(angle)
            y += math.cos(angle)
        return None
    
    
    def check_space(self, current_position):
        lines = self.calculate_lines(current_position,self.angles)

        count = 0
        if lines[0][0]<Parking_height/2 :
            count += 1
        if lines[1][0]<Parking_weight/2 :
            count += 1
        if lines[2][0]<Parking_height/2 :
            count += 1
        if lines[3][0]<Parking_weight/2 :
            count += 1
        if count==0:
            return True
        else:
            return False

    def calculate_lines(self,current_position, angles):
        lines = []
        for angle in angles:
            end = self.find_first_black_pixel(current_position, angle)
            if end is not None:
                length = np.sqrt((end[0] - current_position[0]) ** 2 + (end[1] - current_position[1]) ** 2)
                lines.append((length, end, angle))  # 添加角度到元组
        return lines
    
    # 在那个点上找停车空间
    def process_image(self, x, y, w):
        # 处理图像，找到最长的线段
        # 0.05米一个像素，转换成像素坐标
        # 找1.2米外的点
        self.start = (int(x / pixel_to_meter_ratio),self.img.shape[0]-int(y / pixel_to_meter_ratio))
        self.angles = [w, w + np.pi / 2, w+np.pi, w - np.pi / 2]

        lines = self.calculate_lines(self.start,self.angles)
        max_length, max_end, max_angle = max(lines, key=lambda x: x[0])  # 获取最长线段的角度
        new_start = None


        count = 0
        if lines[0][0]<Parking_height/2 :
            count += 1
        if lines[1][0]<Parking_weight/2 :
            count += 1
        if lines[2][0]<Parking_height/2 :
            count += 1
        if lines[3][0]<Parking_weight/2 :
            count += 1
        
        new_start=None
        if count == 3:
            new_start = (self.start[0] + distance_in_pixels* math.sin(max_angle), self.start[1] + distance_in_pixels * math.cos(max_angle))

        if new_start==None:
            returnnum=None
        else:
            returnnum= (new_start[0]*0.05,(self.img.shape[0]-new_start[1])*0.05)
            print("需要第二目标点")
        return (x, y),returnnum

    def show_image(self, lines, start, new_start):
        # 显示图像，包括找到的线段和点
        img_color = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        start_int = (int(start[0]), int(start[1]))
        if new_start is not None:
            new_start_int = (int(new_start[0]), int(new_start[1]))
            cv2.circle(img_color, new_start_int, 1, (0, 255, 0), -1)
        # huaxian
        for length, end,_ in lines:
            end_int = (int(end[0]), int(end[1]))
            cv2.line(img_color, start_int, end_int, (0, 0, 255), 1)
        cv2.circle(img_color, start_int, 1, (0, 255, 0), -1)

        plt.figure(figsize=(10, 10))
        plt.imshow(cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB), cmap='gray')
        plt.show()





goals=None
global_goal = None
goals_data = None
current_pos= None
current_goal_index = 0
# send_goal_index = 0


def MoveBasecallback(data):
    global global_goal
    global goals_data
    global goals,current_goal_index
    global_goal = {
        'x': data.goal.target_pose.pose.position.x,
        'y': data.goal.target_pose.pose.position.y,
        'z': data.goal.target_pose.pose.orientation.z,
        'w': data.goal.target_pose.pose.orientation.w
    }

    if goals_data!=None and global_goal!=None:
        for goal in goals_data:
            if goal[0]==global_goal['x'] and goal[1]==global_goal['y'] and goal[2]==global_goal['z'] and goal[3]==global_goal['w']:
                return
    current_goal_index=0 #从0开始


    # math.asin((-90/2)*math.pi/180),math.acos((-90/2)*math.pi/180))
    yaw_z = 2*math.atan2(global_goal['z'],global_goal['w'])
    # print(yaw_z)

    processor = ImageProcessor('navigation/src/teb_local_planner_tutorials/maps/maze.png')
    start, new_start = processor.process_image(global_goal['x'],global_goal['y'], yaw_z)
    # processor.show_image(lines, processor.start, new_start)



    #定义四个发送目标点的对象
    # goals = [MoveBaseGoal() for _ in range(2)]
    
    if new_start==None:
        goals = [MoveBaseGoal() for _ in range(1)]
        goals_data=[(start[0],start[1],global_goal['z'],global_goal['w'])]
    else:
        goals = [MoveBaseGoal() for _ in range(2)]
        goals_data=[(new_start[0],new_start[1],global_goal['z'],global_goal['w']) ,
                    (start[0],start[1],global_goal['z'],global_goal['w'])]
    
     
    send_goals_python()




def pose_callback(msg):
    global current_pos,current_goal_index
    # current_pos= msg.pose
    current_pos= msg.pose.pose
    # rospy.loginfo("Robot position: x=%f, y=%f, z=%f" % (current_pos.position.x, current_pos.position.y, current_pos.orientation.z))
    
    if goals==None :
        return
    xy_distance=10
    yaw_distance=10
    xy_distance, yaw_distance = calculate_distance(goals[current_goal_index-1], current_pos)
    print(xy_distance,yaw_distance)
    if xy_distance < 0.4 and yaw_distance < 0.2:  # 0.1 is the tolerance
        send_goals_python()



def calculate_distance(goal, current_position):
    xy_distance = math.sqrt((goal.target_pose.pose.position.x - current_position.position.x)**2 
                     + (goal.target_pose.pose.position.y - current_position.position.y)**2)
    sign = 1 if current_position.orientation.w >= 0 else -1
    yaw_distance = abs(goal.target_pose.pose.orientation.z - sign * current_position.orientation.z)
    return xy_distance, yaw_distance

def send_next_goal(client, goals, current_goal_index):
    if current_goal_index < len(goals):
        goal = goals[current_goal_index]
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        rospy.loginfo("Send NO. %s Goal !!!" % str(current_goal_index))
        return True
    else:
        rospy.loginfo("All goals have been sent.")
        return False

def send_goals_python():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    global goals,goals_data,current_pos,current_goal_index

    print("current_goal_index:",current_goal_index)

    if goals==None:
        return
    if current_goal_index>=len(goals):
        print("完成所有目标点")
        return

    for goal, data in zip(goals, goals_data):
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w = data


    send_next_goal(client, goals, current_goal_index)
    current_goal_index+=1


if __name__ == '__main__':
    rospy.init_node('send_goals_python',anonymous=True)

    # rospy.Subscriber("/amcl_pose", PoseStamped, pose_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, MoveBasecallback)
    # result = send_goals_python()
    # rospy.loginfo(result)
    rospy.spin()


# wait = client.wait_for_result(rospy.Duration.from_sec(8.0))  # 发送完毕目标点之后，根据action 的机制，等待反馈执行的状态，等待时长是：30 s.
        
