"""
# TrajectortRecorder.py
# Decrciption: This module is used to record the trajectory of the robot as a waypoint csv file.
# Author: xzcllwx
# Email: xzcllwx@qq.com
# Date: 20223-12-15
# CopyRight (c) 2023 All rights reserved.
"""

import datetime
import math
import numpy as np
import csv
import os
import sys
import time
# from collections import deque
# import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import transforms3d as tfs
# from starneto_ros_msgs.msg import Gpfpd

class TraRecorder():
    def __init__(self):
        self.data_save_dir = '/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2] + '/' + 'trajectory_record/'
        if not os.path.exists(self.data_save_dir):
            os.makedirs(self.data_save_dir)
        self.data_save_path = self.data_save_dir + 'tra_' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S') + '.csv'

        print('The save path of Log:', self.data_save_path)

        self.record_csv = open(self.data_save_path, 'w', encoding='utf-8')  # Specify the encoding as utf-8
        self.writer = csv.writer(self.record_csv)
        self.writer.writerow(['counter', 'timestamp', 'x(m)', 'y(m)', 'z(m)', 'yaw(°)', 'velocity(km/h)'])
        self.counter = 0
    
    def write_csv(self, ros_timestamp, longtitude, latitude, height, yaw, velocity): # 或者传消息结构体
        self.writer.writerow([self.counter, ros_timestamp, longtitude, latitude, height, yaw, velocity])
        self.counter += 1

    def close_csv(self):
        self.record_csv.close()

# class TraVisual():
#     """
#     This is class for FunctionAnimation in Matplotlib, Get an easy way to update an animation
#     in a specific axes, only for 2-D plot, a 3-D plot hasn't been test!
#     """
#     def __init__(self, ax, plot_element="line", plot_mode="global", max_length=1000, fps=100, **kwargs):

#         self.__ax = ax
#         self.__plot_element = plot_element
#         self.__plot_mode = plot_mode
#         self.__max_length = max_length
#         self.__fps = 1.0/fps

#         if self.__plot_mode == "global":
#             self.__x = deque()
#             self.__y = deque()
#         elif self.__plot_mode == "local":
#             self.__x = deque(maxlen = self.__max_length)
#             self.__y = deque(maxlen = self.__max_length)
#         else:
#             raise TypeError("A wrong mode!")
#         self.__yaw = deque(maxlen = 1)

#         if self.__plot_element == "line":
#             element, = self.__ax.plot(self.__x, self.__y, **kwargs)
#             self.__element = element
#             self.__set_data = self.__element.set_data
#         elif self.__plot_element == "scatter":
#             element = self.__ax.scatter(self.__x, self.__y, **kwargs)
#             self.__element = element
#             self.__set_data = self.__element.set_offsets
#         else:
#             raise TypeError("A wrong plot element!")
#         self.__arrow = self.__ax.arrow(0, 0, 0, 1, width=0.05, color='r')

#     def update_data(self, x, y, yaw):
#         self.__x.append(x)
#         self.__y.append(y)
#         self.__yaw.append(yaw)

#     def update_frame(self):
#         xmin = min(self.__x, default=0) - 0.2 * abs(min(self.__x, default=0))
#         xmax = max(self.__x, default=1) + 0.2 * abs(max(self.__x, default=1))
#         ymin = min(self.__y, default=0) - 0.2 * abs(min(self.__y, default=0))
#         ymax = max(self.__y, default=1) + 0.2 * abs(max(self.__y, default=1))
#         if self.__plot_element == 'scatter':
#             data = np.stack([self.__x, self.__y]).T
#             self.__set_data(data)
#         else:
#             self.__set_data(self.__x, self.__y)
#         self.__ax.set_xlim(xmin, xmax)
#         self.__ax.set_ylim(ymin, ymax)
#         self.__arrow.set_data(x = self.__x[-1], y = self.__y[-1], dx = math.cos(self.__yaw[0]), dy = math.sin(self.__yaw[0]))
#         plt.pause(self.__fps)
    
#     def get_element(self):
#         return len(self.__element)
    
#     def get_dataX(self):
#         return self.__x
    
#     def get_dataY(self):
#         return self.__y
    
#     def get_dataYaw(self):
#         return self.__yaw

def Epsg4326_Epsg3857(longtitude, latitude):
    # Web墨卡托变换。！已测试通过！
    # EPSG:4326 → EPSG:3857
    # unit：degree → m
    radius = 6378137.0 # Earth Radius in Web墨卡托变换
    x = radius * longtitude * math.pi / 180.0
    y = radius * math.log(math.tan((math.pi * (latitude + 90.0)) / 360.0)) # math.log 默认自然底数e
    return x, y

class TraRecorderNode(Node):

    def __init__(self):
        super().__init__('Trajectory_Recorder')
        self.get_logger().info("Recorder/Trajectory Node Initializing.")
        self.traRecorder = TraRecorder()

        # Ros Node
        self.gpsSubscription = self.create_subscription(
            Odometry,
            'odom',
            self.gps_listener,
            10)
        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Recorder/Trajectory Node Initialized.")

    def gps_listener(self, odomMsg):
        # x, y = Epsg4326_Epsg3857(gpsMsg.longitude, gpsMsg.latitude)
        _, _, yaw = tfs.euler.quat2euler([odomMsg.pose.pose.orientation.x, odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z, odomMsg.pose.pose.orientation.w],"sxyz")
        self.traRecorder.write_csv(odomMsg.header.stamp, odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.position.z, yaw, odomMsg.twist.twist.linear.x)
        self.get_logger().info('Recorder heard: x(Lon)-%d, y(Lat)-%d' % (odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y))

def main(args=None):
    rclpy.init(args=args)
    nodeTraRecorder = TraRecorderNode()

    try:
        rclpy.spin(nodeTraRecorder)
    except KeyboardInterrupt:
        rclpy.logging.get_logger("default").info("User requested shutdown.")
    finally:
        nodeTraRecorder.traRecorder.close_csv()
        nodeTraRecorder.destroy_node()
        rclpy.logging.get_logger("default").info("Recorder/Trajectory Node shotdown.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

