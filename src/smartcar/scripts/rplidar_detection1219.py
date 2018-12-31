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
        last_left_dist = 0
        last_right_dist = 0
        for i in range(480, 720):
            # 过滤断点，大于1则判定为不在一条车道线上
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
        z1 = np.polyfit(left_y, left_x, 1)  # 线性拟合
        a1 = z1[0]
        b1 = z1[1]
        road_left_angle = math.atan(-a1 / b1)
        x_left_base = b1
        left_x_pred = []
        # for i in range (0,len(left_y),1):
        #     left_x_pred.append(left_y[i]*a1+b1)
        left_x_pred = np.array(left_y) * a1 + b1
        #print('a1')
        #print(a1)
        #print('b1')
        #print(b1)
        #print('left_y')
        #print(left_y)
        #print ('left_x_pred')
        #print left_x_pred
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
        z2 = np.polyfit(right_y, right_x, 1)  # 线性拟合
        a2 = z2[0]
        b2 = z2[1]
        road_right_angle = math.atan(-a2 / b2)
        road_angle = (road_left_angle + road_right_angle)/2
        expect_angle = road_angle -math.pi
        x_right_base = b2
        right_x_pred = []
        # for i in range (0,len(right_y),1):
        #     right_x_pred.append(right_y[i]*a2+b2)
        right_x_pred = np.array(right_y) * a2 + b2
        #print('right_y')
        #print(right_y)
        #print('right_x_pred')
        #print right_x_pred
        x_offset = (x_right_base-x_left_base)/2

        plt.clf()
        plt.plot(left_x_pred, left_y)
        plt.plot(right_x_pred, right_y)
        plt.axis('equal')
        plt.show()
        plt.pause(0.01)
        return expect_angle,x_offset

    def callback(self, msg):
        angle,offset = self.road_detection(msg)
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
