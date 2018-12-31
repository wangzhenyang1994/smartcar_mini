#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class rplidarnav:
    def __init__(self):
        self.stop_flag = False
        rospy.init_node('rplidar_navigation', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('rp_nav', Twist, queue_size=1)
        print ('rplidar navigation is start')
        plt.ion()
    def road_detection(self, msg):
        #扫描获取左侧雷达数据(左侧60度范围)
        #雷达正前方为720
        left_dist = []  # 左侧激光距离数组
        left_angle = []  # 左侧激光角度数组
        left_x = []  # 左侧x坐标
        left_y = []  # 左侧y坐标
        for i in range(480, 720):
            #最近障碍物小于30cm，则停止
            if msg.ranges[i] < 0.3:
                self.stop_flag = True
            # 过滤断点，大于1则判定为不在一条车道线上
            if msg.ranges[i] < 1 and msg.ranges[i]>0.3:
                current_dist = msg.ranges[i]
                current_angle = (30 + (i - 480)//4)*math.pi//180 #弧度制
                left_dist.append(current_dist)
                left_angle.append(current_angle)
                current_x = -1*current_dist*math.cos(current_angle)
                current_y = current_dist*math.sin(current_angle)
                left_x.append(current_x)
                left_y.append(current_y)
            else:
                continue
        z1 = np.polyfit(left_x, left_y, 1)  # 线性拟合
        a1 = z1[0]
        b1 = z1[1]
        road_left_angle = math.atan(b1 / a1)
        x_left_base = -b1/a1
        left_y_pred = a1 * left_x + b1
        # 扫描获取右侧雷达数据(右侧60度范围)
        right_dist = []  # 右侧激光距离数组
        right_angle = []  # 右侧激光角度数组
        right_x = []  # 右侧x坐标
        right_y = []  # 右侧y坐标
        for i in range(720, 960):
            if msg.ranges[i] < 0.3:
                self.stop_flag = True
            if msg.ranges[i] < 1 and msg.ranges[i]>0.3:
                current_dist = msg.ranges[i]
                current_angle = (30 + (960 - i)//4)*math.pi//180 #弧度制
                right_dist.append(current_dist)
                right_angle.append(current_angle)
                current_x = current_dist*math.cos(current_angle)
                current_y = current_dist*math.sin(current_angle)
                right_x.append(current_x)
                right_y.append(current_y)
            else:
                continue
        z2 = np.polyfit(right_x, right_y, 1)  # 线性拟合
        a2 = z2[0]
        b2 = z2[1]
        road_right_angle = math.atan(b2 / a2)
        expect_angle = (road_left_angle + road_right_angle)/2
        x_right_base = -b2/a2
        right_y_pred = a2 * right_x + b2
        x_offset = x_left_base-x_right_base

        plt.clf()
        plt.plot(left_x, left_y_pred)
        plt.plot(right_x, right_y_pred)
        plt.axis('equal')
        plt.show()
        return expect_angle,x_offset

    def callback(self, msg):
        angle,offset = self.road_detection(msg)
        if self.stop_flag == False:
            print('angle' + angle)
            print('offset' + offset)
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = angle * 1 - offset
        if self.stop_flag == True:
            print('detect obstacle,stop')
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        rp_nav = rplidarnav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
