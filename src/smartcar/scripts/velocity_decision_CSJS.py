#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

class velocityDecide:
    def __init__(self):
        self.sign_flag = 0        #0 means using lane_vel, >0 means using lidar_vel
        self.light_flag = 0
        self.lane_vel = Twist()
        self.lidar_vel = Twist()
        self.cmd_vel = Twist()
        self.has_obs = Bool()
        self.has_green_light = Bool()
        self.has_sign = Bool()
       
        rospy.init_node('velocity_decision', anonymous=True)
        rospy.Subscriber('lane_vel', Twist, self.lanecallback)
        rospy.Subscriber('lidar_vel', Twist, self.lidarcallback)
        rospy.Subscriber('has_obs', Bool, self.obscallback)
        rospy.Subscriber('has_green_light', Bool, self.lightcallback)
        rospy.Subscriber('has_sign', Bool, self.signcallback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
    
    def lanecallback(self, msg):
        self.lane_vel = msg

    def lidarcallback(self, msg):
        self.lidar_vel = msg

    def obscallback(self, msg):
        self.has_obs = msg

    def lightcallback(self, msg):
        self.has_green_light = msg

    def signcallback(self, msg):
        self.has_sign = msg

    def spin(self):
        while not rospy.is_shutdown():

            if self.has_sign.data == True:
                print('***sign detected***')
                self.sign_flag += 1
        
            if self.sign_flag == 0:
                self.cmd_vel = self.lane_vel
            else:
                self.cmd_vel = self.lidar_vel
            
            if self.has_green_light.data == True:
                print('***green light detected***')
                self.light_flag += 1
                if self.light_flag == 0:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = 0
                else:
                    self.cmd_vel.linear.x = 0.4
                    self.cmd_vel.angular.z = 0.03
                    time.sleep(2)   #?????????? How to make it last 3 seconds
            elif self.light_flag <= 1:
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                            
            if self.has_obs.data == True:
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0

            self.pub.publish(self.cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        decide = velocityDecide()
        decide.spin()
    except rospy.ROSInterruptException:
        pass

    
        
