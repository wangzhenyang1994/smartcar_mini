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
        self.srcMat = np.float32([[256, 687], [420, 436], [815, 436], [953, 687]])
        self.dstMat = np.float32([[360, 720], [360, 0], [850, 0], [850, 720]])
        self.M = cv2.getPerspectiveTransform(self.srcMat, self.dstMat)
        self.invM = cv2.getPerspectiveTransform(self.dstMat, self.srcMat)
        
        self.thresh = [0, 128]

        self.cvb = CvBridge()
        self.pub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        rospy.Subscriber('images', Image, self.callback)
        rospy.init_node('laneline_detection', anonymous=True)
        self.signal = 50
        self.count = 30 #检测不到车道线的时间限制为3秒
    
    ## @brief change into binary img
    def threshImg(self, img, thresh = [0, 255]):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img, (5, 5), 2.0)
        binary = np.zeros_like(img)
        binary[(img <= thresh[1]) & (img >= thresh[0])] = 255
        return binary
        
    ## @brief warp the img into top-down view
    def warpImg(self, img):
        warp=cv2.warpPerspective(img, self.M, (img.shape[1],img.shape[0]), flags=cv2.INTER_LINEAR)
        return warp
        
    ## @brief fit left and right lines using np.polyfit()
    def fit_lines(self, binary_img, plot=True):
        print 'Fitting lines...'
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
        if len(leftx) == 0 or len(lefty) == 0 or len(rightx) == 0 or len(righty) == 0:
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
        ym_per_pix = 18. / 14200. # meters per pixel in y dimension
        xm_per_pix = 18. / 18300. # meters per pixel in x dimension
        
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
        return left_curverad, right_curverad, xoffset, angle


    ## @brief warp back into original view
    def warp_perspective_back(self, img, warped, left_fit, right_fit,Minv):
        print 'Warping back...'
        warp_zero = np.zeros_like(warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        ploty=np.linspace(0, img.shape[0]-1, num=img.shape[0])
        #print len(ploty)
        leftx=left_fit[0]*ploty**2+left_fit[1]*ploty+left_fit[2]
        rightx=right_fit[0]*ploty**2+right_fit[1]*ploty+right_fit[2]
        pts_left = np.array([np.transpose(np.vstack([leftx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([rightx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        #print pts

        ##== Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
        #cv2.imshow('warp', color_warp)

        ##== Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (img.shape[1], img.shape[0])) 
        ##== Combine the result with the original image
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
        return result

    def callback(self, imgmsg):
        img = self.cvb.imgmsg_to_cv2(imgmsg)

        ##== Warp
        binary = self.threshImg(img, self.thresh)
        warp = self.warpImg(binary)
        #cv2.imshow('binary', binary)
        #cv2.imshow('warp', warp)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #cv2.imwrite('warp.jpg', warp)
        if self.count <= 0:#连续3秒检测不到车道线则让车辆停止
            twist=Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.pub.publish(twist)

        ##== Fit
        if self.signal > 0:
            ret, left_fit, right_fit, out_img = self.fit_lines(warp, False)
            if ret == False: 
                cv2.imshow('laneline', result)
                #cv2.imshow('laneline', warp)
                cv2.waitKey(1)  
                return
            else:
                self.left_fit = left_fit
                self.right_fit = right_fit
                self.signal -= 1
        else:
            ret, left_fit, right_fit, out_img = self.fit_lines_continous(warp, self.left_fit, self.right_fit, nwindows=9,plot=False)
            if ret == False:
                self.count -= 1
                cv2.imshow('laneline', result)
                #cv2.imshow('laneline', warp)
                cv2.waitKey(1)
                return
            else:
                self.left_fit = left_fit
                self.right_fit = right_fit               
                if self.count < 30:
                     self.count += 1 

        #plt.show()
        #print left_fit, right_fit
        
        ##== Calculate curvature
        left_curverad, right_curverad, xoffset, angle = self.cal_curverature(img.shape, self.left_fit, self.right_fit)
        cur = (left_curverad+right_curverad) / 2.0

        ##== Warp back
        result = self.warp_perspective_back(img, warp, self.left_fit, self.right_fit, detector.invM)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(result, 'Radius of Curvature = %d(m)' % (cur), (50, 50), font, 1, (255, 0, 0), 2)
        cv2.putText(result, 'Respective angle = %d(deg)' % int(angle / 3.14 * 180), (50, 100), font, 1, (255, 0, 0), 2)
        direction = 'left' if xoffset>0 else 'right'
        cv2.putText(result, ('Vehicle is at %.2f m %s of the center' % (np.abs(xoffset),direction)), (50, 150), font, 1, (255, 0, 0), 2)
        
        #plt.figure(figsize=(10,8))
        cv2.imshow('laneline', result)
        #cv2.imshow('laneline', warp)
        cv2.waitKey(1)
        
        ##== publish msg to /cmd_vel
        if self.signal <= 0:
            twist=Twist()
            twist.linear.x = 0.2
            twist.angular.z = angle * 1.5 - xoffset
            self.pub.publish(twist)



	        #rospy.loginfo(Line_Info)
    
if __name__ == '__main__':
    try:
        detector = lanelineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()






