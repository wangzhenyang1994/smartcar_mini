#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 18 20:51:41 2018

@author: jjldr
"""
import os
import sys
import glob
import cv2 as cv

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def template_demo(target,tpl):
    
    #target = cv.imread("E:/imageload/target1.jpg")
    #cv.namedWindow('template image', cv.WINDOW_NORMAL)
    #cv.imshow("template image", tpl)
    #cv.namedWindow('target image', cv.WINDOW_NORMAL)
    #cv.imshow("target image", target)
    #methods = [cv.TM_SQDIFF_NORMED, cv.TM_CCORR_NORMED, cv.TM_CCOEFF_NORMED]   #3种模板匹配方法
    methods=cv.TM_CCOEFF_NORMED
    th, tw = tpl.shape[:2]
    
    
    result = cv.matchTemplate(target, tpl, methods)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
    if max_val < 0.7:
        return target, False
    if methods == cv.TM_SQDIFF_NORMED:
        tl = min_loc
    else:
        tl = max_loc
    br = (tl[0]+tw, tl[1]+th)   #br是矩形右下角的点的坐标
    cv.rectangle(target, tl, br, (0, 0, 255), 2)
    return target, True
    
class trafficLightDetector:
    def __init__(self):
        self.has_green_light = Bool()
        self.has_sign = Bool()
        self.cvb = CvBridge()
        currentpath, _ = os.path.split(os.path.abspath(sys.argv[0]))
        self.lightpath = os.path.join(currentpath, 'template/light_sample.png')
        self.signpath = os.path.join(currentpath, 'template/sign_sample.png')
        self.lightpub = rospy.Publisher('has_green_light', Bool, queue_size=1)
        self.signpub = rospy.Publisher('has_sign', Bool, queue_size=1)
        rospy.Subscriber('images', Image, self.callback)
        rospy.init_node('traffic_light_detection', anonymous=True)
        
    def callback(self, imgmsg):
        img = self.cvb.imgmsg_to_cv2(imgmsg)
        lighttpl =cv.imread(self.lightpath)
        signtpl = cv.imread(self.signpath)
        light_result, has_green_lights = template_demo(img, lighttpl)
        sign_result, has_sign = template_demo(img, signtpl)
        self.has_green_light.data = has_green_lights
        self.has_sign.data = has_sign
        self.lightpub.publish(self.has_green_light)
        self.signpub.publish(self.has_sign)
        #cv.imshow("tracffic_light", light_result)
        #cv.waitKey(1)

if __name__ == "__main__":
    try:
        detector = trafficLightDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
