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
K=1
L=0.8


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
        self.xoffset_array = [0]
        self.angle_array = [0]

        self.cvb = CvBridge()
        self.pub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        rospy.Subscriber('images', Image, self.callback)
        rospy.init_node('laneline_detection', anonymous=True)
        self.flag=0
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

    def processimg(self, img):
        upper = 15
        lower = 718
        upper_num = 0.1
        lower_num = 0.1
        upper_index_sum = 0
        lower_index_sum = 0

        # ---------------------#
        thresh=100

        histogram=np.sum(img[(img.shape[0]//10)*9:,:],axis=0)
        #out_img = np.dstack((img, img, img))
        #out_img=np.uint8(out_img*255)
        #out_img=255 * out_img
        midpoint=histogram.shape[0]//2
        left_base=np.argmax(histogram[:midpoint])
        right_base=np.argmax(histogram[midpoint:])+midpoint


        #---------------------#
        upper_avg=640

        for i in range(1280):
            if (img[upper, i] == 255):
                upper_num = upper_num + 1
                upper_index_sum = upper_index_sum + i
#            if (img[lower, i] == 255):
#                lower_num = lower_num + 1
#                lower_index_sum = lower_index_sum + i
            upper_avg = upper_index_sum / upper_num
#            lower_avg = lower_index_sum / lower_num

        # ---------------------#
        target=640

        if (left_base>thresh)&(right_base>thresh):
            self.flag=0
        elif ((self.flag==0)or(self.flag==1))and(left_base>thresh)&(right_base<thresh):
            self.flag=1
            target = upper_avg + HALFWIDTH
        elif ((self.flag==0)or(self.flag==2))and(left_base<thresh)&(right_base>thresh):
            self.flag=2
            target = upper_avg - HALFWIDTH

        if target>=1280:
            target=1280
        elif target<0:
            target=0

        print('left_base:',left_base)
        print('right_base:',right_base)
        print('upper_avg:',upper_avg)
        print('target:',target)
        print('self.flag:',self.flag)


        el=target-640

        return el





    def cal_delta(self,el,VX,K,L):
        #K ∝ VX
        return math.degrees(math.atan((2*L*el)/(K*(VX**2))))

    def trans(self,delta):
        #turn right delta>=0;turn left delta<0
        return -delta*0.02


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
        angle=0

        
        direction='straight'
        if self.flag==1:
            direction='rightturn'
        elif self.flag==2:
            direction=='leftturn'

       
        if self.signal > 0:
            self.signal -= 1
        ##== publish msg to /cmd_vel
        if self.signal <= 0:
            twist = Twist()
            twist.linear.x = VX

            delta=self.cal_delta(el,VX,K,L)
            angle=self.trans(delta)

            print('\n\nangle:' + str(angle) + '\nel:' + str(el))
            twist.angular.z =angle
            self.pub.publish(twist)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(warp, 'angle = %f(rad)' % angle, (50, 100), font, 1, (255, 0, 0), 2)
        cv2.putText(warp, (' %.2f m %s of the center' % (np.abs(el), direction)), (50, 150), font, 1, (255, 0, 0),2)
        cv2.imshow('warp', warp)
        cv2.waitKey(1)


            # rospy.loginfo(Line_Info)

if __name__ == '__main__':
    try:
        detector = lanelineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

    """    
    ## @brief fit left and right lines using np.polyfit()
    def fit_lines(self, binary_img, plot=True):
        #print 'Fitting lines...'
        histogram=np.sum(binary_img[binary_img.shape[0]//2:,:],axis=0)
        out_img = np.dstack((binary_img, binary_img, binary_img))
        out_img=np.uint8(out_img*255)
        out_img=255 * out_img
        midpoint=histogram.shape[0]//2
        left_base=np.argmax(histogram[:midpoint])
        right_base=np.argmax(histogram[midpoint:])+midpoint
        nwindows=9
        window_height = np.int(binary_img.shape[0]/nwindows)
        
        ##== Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_current=left_base
        right_current=right_base
        margin=80
        minpix=55
        left_lane_inds=[]
        right_lane_inds=[]
        
        for window in range(nwindows):
            win_y_low = binary_img.shape[0] - (window+1)*window_height
            win_y_high = binary_img.shape[0] - window*window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
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

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)


        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        #print leftx, lefty, rightx, righty 
        '''	
        if len(leftx) == 0 or len(lefty) == 0 or len(rightx) == 0 or len(righty) == 0:
            return False, [], [], []
        '''
        if len(leftx) == 0 or len(lefty) == 0:
            self.left_turn = True
            print('left***********************************')
            return False, [], [], []
        if len(rightx) == 0 or len(righty) == 0:
            self.right_turn = True
            print('right***********************************')
            return False, [], [], []

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        ploty = np.linspace(0, binary_img.shape[0]-1, binary_img.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
        if plot==True:
            fig = plt.figure()
            plt.imshow(out_img)
            plt.plot(left_fitx, ploty, color='yellow')
            plt.plot(right_fitx, ploty, color='yellow')
            plt.xlim(0, 1280)
            plt.ylim(720, 0)
        return True, left_fit, right_fit, out_img
    
    ## @brief fit lines with data from last time
    def fit_lines_continous(self, binary_warped, left_fit, right_fit, nwindows=9,plot=True):
        print 'Fitting lines continous...'
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 80
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & 
                          (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & 
                           (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))  

        ##== Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        if len(leftx) == 0 or len(lefty) == 0 or len(rightx) == 0 or len(righty) == 0:
            return False, [], [], []

        ##== Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        ##== Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        window_img = np.zeros_like(out_img)
        ##== Color in left and right line pixels
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        ##== Generate a polygon to illustrate the search window area
        ##== And recast the x and y points into usable format for cv2.fillPoly()
        left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
        left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, ploty])))])
        left_line_pts = np.hstack((left_line_window1, left_line_window2))
        right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
        right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
        right_line_pts = np.hstack((right_line_window1, right_line_window2))
        if plot==True:
            ##== Draw the lane onto the warped blank image
            cv2.fillPoly(out_img, np.int_([left_line_pts]), (0,255, 0))
            cv2.fillPoly(out_img, np.int_([right_line_pts]), (0,255, 0))
            plt.imshow(out_img)
            plt.plot(left_fitx, ploty, color='yellow')
            plt.plot(right_fitx, ploty, color='yellow')
            plt.xlim(0, 1280)
            plt.ylim(720, 0)
        return True,left_fit, right_fit, out_img
        
        
    ## @brief calculate the mean curverature
    def cal_curverature(self, img_shape, left_fit, right_fit):
        print 'Calculating Curvature...'
        
        ploty=np.linspace(0, img_shape[0]-1, num=img_shape[0])
        ym_per_pix = 0.77 / 659. # meters per pixel in y dimension
        xm_per_pix = 0.43 / 520. # meters per pixel in x dimension
        
        ##== Fit new polynomials to x,y in world space
        y_eval = np.max(ploty)
        leftx=left_fit[0]*ploty**2+left_fit[1]*ploty+left_fit[2]
        rightx=right_fit[0]*ploty**2+right_fit[1]*ploty+right_fit[2]
        left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

        ##== Calculate the new radii of curvature
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        xoffset=(left_fit_cr[2]+right_fit_cr[2])/2-img_shape[1]*xm_per_pix/2
        
        ##== calculate angle
        #print left_fit, right_fit
        left_angle = math.atan(2 * left_fit[0] * y_eval + left_fit[1])
        right_angle = math.atan(2 * right_fit[0] * y_eval + right_fit[1])
        angle = (left_angle + right_angle) / 2.0
        #print angle

        xoffset_blur = 0
        angle_blur = 0
        print('xoffset')
        print(xoffset)
        print('angle')
        print(angle)
        self.xoffset_array.append(xoffset)
        self.angle_array.append(angle)
        print('len')
        print(len(self.xoffset_array))
        coef = []
        for j in range(180):
            coef.append(np.exp(-0.005*j))


        if len(self.xoffset_array) <= 180:
            print('start v1')
            return left_curverad, right_curverad, xoffset, angle
        else:
            print('v2')
            angle_array_latest = angle_array[-180, -1]
            for i in range(180):
                angle_array_latest[i] = angle_array_latest[i] * coef[i]
                angle_blur = angle_blur + angle_array_latest[i]
            #for i in range(len(self.xoffset_array)-1,len(self.xoffset_array)-180,-1):
            #    angle_array[i] = coef[i] * angle_array[i]
                
            #    angle_blur += self.angle_array[i]
                

            angle_blur = angle_blur/180

            print('angle_blur')
            print(angle_blur)
            return left_curverad, right_curverad, xoffset, angle_blur """

