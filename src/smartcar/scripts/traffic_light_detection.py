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
        self.has_red_light = Bool()
        self.cvb = CvBridge()
        currentpath, _ = os.path.split(os.path.abspath(sys.argv[0]))
        self.tplpath = os.path.join(currentpath, 'template/sample.jpg')
        self.pub = rospy.Publisher('has_red_light', Bool, queue_size=1)
        rospy.Subscriber('images', Image, self.callback)
        rospy.init_node('traffic_light_detection', anonymous=True)
        
    def callback(self, imgmsg):
        img = self.cvb.imgmsg_to_cv2(imgmsg)
        tpl =cv.imread(self.tplpath)
        result, has_red_lights = template_demo(img, tpl)
        self.has_red_light.data = has_red_lights
        self.pub.publish(self.has_red_light)
        cv.imshow("tracffic_light", result)
        cv.waitKey(1)

if __name__ == "__main__":
    try:
        detector = trafficLightDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
