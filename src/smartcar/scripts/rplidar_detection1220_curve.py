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
    def road_detection_all(self,msg):
        #雷达正前方为720,以线在前
        left_dist = []  # 左侧激光距离数组
        left_angle = []  # 左侧激光角度数组
        left_x = []  # 左侧x坐标
        left_y = []  # 左侧y坐标
        right_dist = []  # 右侧激光距离数组
        right_angle = []  # 右侧激光角度数组
        right_x = []  # 右侧x坐标
        right_y = []  # 右侧y坐标
        break_point_index = 0 #分界点坐标索引
        #找左右车道分界点
        #扫描180度范围角度10米内的障碍物
        last_dist = 100
        for i in range(360,480):
            if msg.ranges[i]<last_dist:
                last_dist = msg.ranges[i]
            else:
                continue
        for i in range (360,1080):
            if msg.ranges[i]<10:
                current_dist = msg.ranges[i]
                #两次扫猫距离在1m以上，两个点不在同一侧车道线
                if math.fabs(last_dist-current_dist)>1 and math.fabs(last_dist-current_dist)<50:
                    break_point_index = i
                    break
                else:
                    last_dist = current_dist
                    continue
            else:
                continue
        #左侧车道线拟合
        for i in range (360,break_point_index):
            current_angle = (0 + (i - 360)/4)*math.pi/180 #弧度制
            left_dist.append(current_dist)
            left_angle.append(current_angle)
            if current_angle <= math.pi/2:
                current_x = -1*current_dist*math.cos(current_angle)
                current_y = current_dist*math.sin(current_angle)
                left_x.append(current_x)
                left_y.append(current_y)
            if current_angle > math.pi/2:
                current_angle = math.pi - current_angle
                current_x = current_dist*math.cos(current_angle)
                current_y = current_dist*math.sin(current_angle)
                left_x.append(current_x)
                left_y.append(current_y)
        z1 = np.polyfit(left_y, left_x, 2)  # 二次拟合
        a1 = z1[0]
        b1 = z1[1]
        c1 = z1[2]
        left_x_pred = []
        x_left_base = c1
        #计算对应y的x坐标 x=a1*y^2+b1*y+c1
        for i in range (0,len(left_y),1):
            left_x_pred.append(a1*left_y[i]**2+b1*left_y[i]+c1)
        #计算切线斜率和道路偏向角
        road_left_angle = math.atan(2 * a1 * c1 + b1)        
        #右侧车道拟合
        for i in range (break_point_index,1080):
            current_angle = (0 + (1080 - i)/4)*math.pi/180 #弧度制
            right_dist.append(current_dist)
            right_angle.append(current_angle)
            if current_angle <= math.pi/2:
                current_x = current_dist*math.cos(current_angle)
                current_y = current_dist*math.sin(current_angle)
                right_x.append(current_x)
                right_y.append(current_y)
            if current_angle > math.pi/2:
                current_angle = math.pi - current_angle
                current_x = -1*current_dist*math.cos(current_angle)
                current_y = current_dist*math.sin(current_angle)               
                right_x.append(current_x)
                right_y.append(current_y)
        z2 = np.polyfit(right_y, right_x, 1)  # 二次拟合
        a2 = z2[0]
        b2 = z2[1]
        c2 = z2[2]
        x_right_base = c2
        right_x_pred = []
        for i in range (0,len(right_y),1):
            right_x_pred.append(a2*right_y[i]**2+b2*right_y[i]+c2)
        road_right_angle = math.atan(2 * a2 * c2 + b2)
        #计算道路倾角
        road_angle = (road_left_angle + road_right_angle)/2
        #计算离道路中心的偏移
        left_near_dist = 0
        right_near_dist = 0
        left_near = []
        right_near = []
        for i in range(360,480):
            if msg.ranges[i]<3:
                left_near.append(msg.ranges[i])
            else:
                continue
        left_near_dist = np.min(left_near)
        for i in range(960,1080):
            if msg.ranges[i]<3:
                right_near.append(msg.ranges[i])
            else:
                continue
        right_near_dist = np.min(right_near)
        x_offset = (right_near_dist-left_near_dist)/2# 大于零右偏，小于零左偏
        plt.clf()
        plt.plot(left_x_pred, left_y, color='blue')
        plt.plot(right_x_pred, right_y,color = 'red')
        plt.axis('equal')
        plt.show()
        plt.pause(0.01)
        return road_angle,x_offset

    def road_detection(self, msg):
        #扫描获取左侧雷达数据(左侧60度范围)
        #雷达正前方为720
        left_dist = []  # 左侧激光距离数组
        left_angle = []  # 左侧激光角度数组
        left_x = []  # 左侧x坐标
        left_y = []  # 左侧y坐标
        last_left_dist = 0
        last_right_dist = 0
        for i in range(480, 720):
            # 过滤断点，大于1.5则判定为不在一条车道线上
            if msg.ranges[i] < 1.5:
                current_dist = msg.ranges[i]
                if math.fabs(last_left_dist-current_dist)<=0.05:
                    current_angle = (30 + (i - 480)/4)*math.pi/180 #弧度制
                    left_dist.append(current_dist)
                    left_angle.append(current_angle)
                    current_x = -1*current_dist*math.cos(current_angle)
                    current_y = current_dist*math.sin(current_angle)
                    left_x.append(current_x)
                    left_y.append(current_y)
                    last_left_dist = current_dist
                else:
                    last_left_dist = current_dist
            else:
                continue
        z1 = np.polyfit(left_y, left_x, 2)  # 二次拟合
        a1 = z1[0]
        b1 = z1[1]
        c1 = z1[2]
        left_x_pred = []
        x_left_base = c1
        #计算对应y的x坐标 x=a1*y^2+b1*y+c1
        for i in range (0,len(left_y),1):
            left_x_pred.append(a1*left_y[i]**2+b1*left_y[i]+c1)
        #计算切线斜率和道路偏向角
        #y_eval = np.max(left_y)
        road_left_angle = math.atan(2 * a1 * c1 + b1)
        # 扫描获取右侧雷达数据(右侧60度范围)
        right_dist = []  # 右侧激光距离数组
        right_angle = []  # 右侧激光角度数组
        right_x = []  # 右侧x坐标
        right_y = []  # 右侧y坐标
        for i in range(960, 720,-1):
            if msg.ranges[i] < 1.5:
                current_dist = msg.ranges[i]
                if math.fabs(last_right_dist-current_dist)<=0.05:
                    current_dist = msg.ranges[i]
                    current_angle = (30 + (960 - i)/4)*math.pi/180 #弧度制
                    right_dist.append(current_dist)
                    right_angle.append(current_angle)
                    current_x = current_dist*math.cos(current_angle)
                    current_y = current_dist*math.sin(current_angle)
                    right_x.append(current_x)
                    right_y.append(current_y)
                    last_right_dist = current_dist
                else:
                    last_right_dist = current_dist
            else:
                continue
        z2 = np.polyfit(right_y, right_x, 1)  # 二次拟合
        a2 = z2[0]
        b2 = z2[1]
        c2 = z2[2]
        x_right_base = c2
        right_x_pred = []
        for i in range (0,len(right_y),1):
            right_x_pred.append(a2*right_y[i]**2+b2*right_y[i]+c2)
        road_right_angle = math.atan(2 * a2 * c2 + b2)
        road_angle = (road_left_angle + road_right_angle)/2
        expect_angle = road_angle-math.pi/2#大于零左转，小于零右转
        x_offset = (x_right_base-x_left_base)/2

        plt.clf()
        plt.plot(left_x_pred, left_y)
        plt.plot(right_x_pred, right_y)
        plt.axis('equal')
        plt.show()
        plt.pause(0.01)
        return expect_angle,x_offset

    def callback(self, msg):
        angle,offset = self.road_detection_all(msg)
        if self.stop_flag == False:
            print('angle')
            print(angle)
            print('offset')
            print(offset)
            twist = Twist()
            twist.linear.x = 0.3
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

