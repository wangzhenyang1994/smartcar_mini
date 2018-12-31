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


VX=0.3
K=0.1
L=0.8
HALFWIDTH=320

class lanelineDetector():

    def __init__(self):
        self.srcMat = np.float32([[256, 687], [430, 436], [805, 436], [953, 687]])
        self.dstMat = np.float32([[360, 720], [360, 0], [850, 0], [850, 720]])
        self.M = cv2.getPerspectiveTransform(self.srcMat, self.dstMat)
        self.invM = cv2.getPerspectiveTransform(self.dstMat, self.srcMat)

        self.left_turn = False
        self.right_turn = False
        self.thresh = [0, 128]

        self.cvb = CvBridge()
        self.pub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        rospy.Subscriber('images', Image, self.callback)
        rospy.init_node('laneline_detection', anonymous=True)
        self.flag = 0

        self.threshsum=255*5
        self.midthresh=50
        self.signal = 20

        self.upperlineupbound=50
        self.upperlinelowbound=100

        self.whichlinelist=['both']

        self.whichline='both'
        self.direction='forward'
        self.directionthresh=0.5
       # self.target=640

        self.left_final = 320
        self.right_final = 960



    def threshImg(self, img, thresh=[0, 255]):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 2.0)
        gray = cv2.medianBlur(gray, 5)
        retval, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
        binary = 255 - binary
    # binary = np.zeros_like(img)
    # binary[(img <= thresh[1]) & (img >= thresh[0])] = 255
        return binary


    def warpImg(self, img):
        warp = cv2.warpPerspective(img, self.M, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR)
        return warp


    def fit_lines(self,binary_img,left_base,right_base):
        print 'Fitting lines...'
        histogram = np.sum(binary_img[binary_img.shape[0] // 2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        nwindows = 9
        window_height = np.int(binary_img.shape[0] / nwindows)

        out_img = np.dstack((binary_img, binary_img, binary_img))
        out_img = np.uint8(out_img * 255)
        out_img = 255 * out_img

    ##== Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_current = left_base
        right_current = right_base
        margin = 150
        minpix = 55
        left_lane_inds = []
        right_lane_inds = []

        left_fit=[]
        right_fit=[]

        for window in range(nwindows):
            win_y_low = binary_img.shape[0] - (window + 1) * window_height
            win_y_high = binary_img.shape[0] - window * window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)



            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) &
                          (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) &
                           (nonzerox < win_xright_high)).nonzero()[0]
            ##== Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            ##== If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                left_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                right_current = np.int(np.mean(nonzerox[good_right_inds]))
            self.left_final=left_current
            self.right_final=right_current

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
    # print leftx, lefty, rightx, righty


        ploty = np.linspace(0, binary_img.shape[0] - 1, binary_img.shape[0])
        plt.imshow(out_img)

        if (len(leftx) > 0)and(len(lefty)>0):
            left_fit = np.polyfit(lefty, leftx, 2)
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            plt.plot(left_fitx, ploty, color='yellow')

        if (len(rightx) > 0)and(len(righty)>0):
            right_fit = np.polyfit(righty, rightx, 2)
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
            plt.plot(right_fitx, ploty, color='yellow')


        plt.xlim(0, 1280)
        plt.ylim(720, 0)

        print(left_fit)
        print(right_fit)

        return left_fit, right_fit,out_img


## @brief fit lines with data from last time
    def fit_lines_continous(self,binary_warped,  left_fit, right_fit, nwindows=9, plot=True):
        print 'Fitting lines continous...'
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 80

        if len(left_fit)>0:
            left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] - margin)) &
                      (nonzerox < (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] + margin)))
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            self.left_final=np.mean(leftx)

            left_fit = np.polyfit(lefty, leftx, 2)

        if len(right_fit)>0:
            right_lane_inds = ((nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] - margin)) &
                       (nonzerox < (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] + margin)))
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
            right_fit = np.polyfit(righty, rightx, 2)
            self.right_final=np.mean(rightx)


        return left_fit, right_fit


    def processimg(self, binary_img):

    # ---------------------#
        cv2.imshow('image', binary_img)
        histogram = np.sum(binary_img[binary_img.shape[0] // 20 * 19:, :], axis=0)
        upperhistogram = np.sum(binary_img[self.upperlineupbound:self.upperlinelowbound, :], axis=0)

        midpoint = histogram.shape[0] // 2
        if np.max(histogram[:midpoint]) >= self.threshsum:
            left_base = np.argmax(histogram[:midpoint])
        else:
            left_base = -1

        if np.max(histogram[midpoint:]) + midpoint >= self.threshsum:
            right_base = np.argmax(histogram[midpoint:]) + midpoint
        else:
            right_base = -1

    # ---------------------#
        if self.signal > 0:
            self.left_fit, self.right_fit, img = self.fit_lines(binary_img, left_base if left_base != -1 else 320,
                                             right_base if right_base != -1 else 960)
            cv2.imshow('laneline', img)
            cv2.waitKey(1)


        else:
            self.left_fit, self.right_fit = self.fit_lines_continous(binary_img,self.left_fit, self.right_fit, nwindows=9, plot=False)

        fitl = 0
        fitr = 0
        if (len(self.left_fit) > 0):
            fitl = self.left_fit[1]
        if (len(self.right_fit) > 0):
            fitr = self.right_fit[1]
        fit = (fitl + fitr) / 2

        if fit >= self.directionthresh:
            self.direction = 'left'
        if fit <= -self.directionthresh:
            self.direction = 'right'
        if -self.directionthresh < fit < self.directionthresh:
            self.direction = 'forward'

    # decide which line
        if (midpoint - left_base <= self.midthresh) and (right_base - midpoint <= self.midthresh):
        # the line is in the middle
            pass
        else:
            if (left_base != -1) and (right_base != -1):
            # both lines can be seen
                self.whichline = 'both'
            elif (left_base != -1) and (self.whichlinelist[-1] == 'both'):
                self.whichline = 'left'
            elif (right_base != -1) and (self.whichlinelist[-1] == 'both'):
                self.whichline = 'right'
            self.whichlinelist.append(self.whichline)

        print(left_base, ' ', right_base)
        print(self.direction)
        print(self.whichlinelist)


    # ---------------------#



        if self.direction=='forward':
            if self.whichline=='left':
                self.target = self.left_final + HALFWIDTH
            else:
                self.target = self.right_final - HALFWIDTH
        elif self.direction=='left':
            if self.whichline=='left':
                self.target = self.left_final + HALFWIDTH
            else:
                self.target = self.right_final - HALFWIDTH
        elif self.direction=='right':
            if self.whichline=='left':
                self.target = self.left_final + HALFWIDTH
            else:
                self.target = self.right_final - HALFWIDTH

        if self.target >= 1280:
            self.target = 1280
        elif self.target < 0:
            self.target = 0

        print('left_base:', left_base)
        print('right_base:', right_base)
        print('target:', self.target)
        print('self.flag:', self.flag)

        el = self.target - 640

        return el


    def cal_delta(self, el, VX, K, L):
    # K  VX
        return math.degrees(math.atan((2 * L * el/100) / ((K * VX) ** 2)))


    def trans(self, delta):
    # turn right delta>=0;turn left delta<0
        return -delta * 0.02


    def callback(self, imgmsg):
        img = self.cvb.imgmsg_to_cv2(imgmsg)
        k = 0
    ##== Warp
        binary = self.threshImg(img, self.thresh)
        warp = self.warpImg(binary)
    # cv2.imshow('binary', binary)
    # cv2.imshow('wjjuujju	oyAllWindows()
    # cv2.imwrite('warp.jpg', warp)
        el = self.processimg(warp)
        angle = 0



        if self.signal > 0:
            self.signal -= 1
    ##== publish msg to /cmd_vel
        if self.signal <= 0:
            twist = Twist()
            twist.linear.x = VX

            delta = self.cal_delta(el, VX, K, L)
            angle = self.trans(delta)

            print('\n\nangle:' + str(angle) + '\nel:' + str(el))
            twist.angular.z = angle
            self.pub.publish(twist)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(warp, 'angle = %f(rad)' % delta, (50, 100), font, 1, (255, 0, 0), 2)
        cv2.putText(warp, (' %.2f m %s of the center' % (np.abs(el), self.direction)), (50, 150), font, 1, (255, 0, 0), 2)
        cv2.imshow('warp', warp)
        cv2.waitKey(1)







##== Fit






if __name__ == '__main__':
    try:
        detector = lanelineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
