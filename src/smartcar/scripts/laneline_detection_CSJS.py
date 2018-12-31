#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import math
import cv2
import glob
import rospy
import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

## @brief detect the lane lines
class lanelineDetector():

    ## @brief initialize the object
    def __init__(self):    
        self.srcMat = np.float32([[256, 687], [430, 436], [805, 436], [953, 687]])
        self.dstMat = np.float32([[360, 720], [360, 0], [850, 0], [850, 720]])
        self.M = cv2.getPerspectiveTransform(self.srcMat, self.dstMat)
        self.invM = cv2.getPerspectiveTransform(self.dstMat, self.srcMat)
        
        self.left_turn=False
        self.right_turn=False
        self.thresh = [0, 128]
        self.offset_array = [0]
        self.angle_array = [0]

        self.cvb = CvBridge()
        self.pub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        rospy.Subscriber('images', Image, self.callback)
        rospy.init_node('laneline_detection', anonymous=True)
        self.signal = 50
        #self.count = 30 #检测不到车道线的时间限制为3秒
    
    ## @brief change into binary img
    def threshImg(self, img, thresh = [0, 255]):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 2.0)
        gray = cv2.medianBlur(gray,5)
        retval, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
        binary = 255 - binary
        #binary = np.zeros_like(img)
        #binary[(img <= thresh[1]) & (img >= thresh[0])] = 255
        return binary
        
    ## @brief warp the img into top-down view
    def warpImg(self, img):
        warp=cv2.warpPerspective(img, self.M, (img.shape[1],img.shape[0]), flags=cv2.INTER_LINEAR)
        return warp


    def cal_delta(self, img):
        upper = 1
        lower = 718
        upper_num = 0.1
        lower_num = 0.1
        upper_index_sum = 0
        lower_index_sum = 0
        angle_blur = 0
        offset_blur = 0
        for i in range(1280):
            if(img[upper, i] == 255):
                upper_num = upper_num + 1
                upper_index_sum = upper_index_sum + i
            if(img[lower, i] == 255):
                lower_num = lower_num + 1
                lower_index_sum = lower_index_sum + i
        upper_avg = upper_index_sum / upper_num
        lower_avg = lower_index_sum / lower_num
        # angle 是方向打的角度，<0需要向左打，>0向右打
        angle = math.atan((upper_avg - lower_avg)/200.)
        # offset 是与车道的偏离程度，<0时向左打，>0向右打
        offset = (lower_avg - 640.) / 100.
        self.angle_array.append(angle)
        self.offset_array.append(offset)
        if len(self.angle_array)< 10:
            return angle,offset
        else:
            for i in range(len(self.angle_array)-1,len(self.angle_array)-10,-1):
                offset_blur += self.offset_array[i]
                offset_blur = offset_blur/10
                angle_blur += self.angle_array[i]
                angle_blur = angle_blur/10
                self.angle_array.pop(0)
        return angle_blur, offset_blur


    def callback(self, imgmsg):
        img = self.cvb.imgmsg_to_cv2(imgmsg)
        k = 0
        ##== Warp
        binary = self.threshImg(img, self.thresh)
        warp = self.warpImg(binary)
        #cv2.imshow('binary', binary)
        #cv2.imshow('wjjuujju	oyAllWindows()
        #cv2.imwrite('warp.jpg', warp)
        angle, xoffset = self.cal_delta(warp)
        if abs(angle) <= 0.2:
            k = 0.2
        #font = cv2.FONT_HERSHEY_SIMPLEX
        #cv2.putText(warp, 'angle = %f(rad)' % angle, (50, 100), font, 1, (255, 0, 0), 2)
        #direction = 'left' if xoffset>0 else 'right'
        #cv2.putText(warp, (' %.2f m %s of the center' % (np.abs(xoffset),direction)), (50, 150), font, 1, (255, 0, 0), 2)
        #out_img = np.dstack((warp, warp, warp))
        #cv2.imwrite(str(self.i).zfill(6)+".png", out_img)
        cv2.imshow('warp', warp)
        
        cv2.waitKey(1)
        
        
        if self.signal > 0:
            self.signal -= 1
        ##== publish msg to /cmd_vel
        if self.signal <= 0:
            twist=Twist()
            twist.linear.x = 0.25

            angle = - angle * 1.
            xoffset = - xoffset * k

            #print('\n\nangle:' + str(angle) + '\nxoffset:' + str(xoffset))
            twist.angular.z = angle + xoffset
            self.pub.publish(twist)



	        #rospy.loginfo(Line_Info)
    
if __name__ == '__main__':
    try:
        detector = lanelineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
