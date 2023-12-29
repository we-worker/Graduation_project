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
            x += np.cos(angle)
            y += np.sin(angle)
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
        self.start = (int(x / pixel_to_meter_ratio), self.img.shape[0]-int(y / pixel_to_meter_ratio))
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
            new_start = (self.start[0] + distance_in_pixels* np.cos(max_angle), self.start[1] + distance_in_pixels * np.sin(max_angle))

        return lines, max_length, max_end, new_start


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

# # 使用示例  
processor = ImageProcessor('navigation/src/teb_local_planner_tutorials/maps/maze.png')
lines, max_length, max_end, new_start = processor.process_image(3.75,2, 3.1415926/2)
processor.show_image(lines, processor.start, new_start)
