#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class velocityDecide:
    def __init__(self):
        self.lane_vel = Twist()
        self.rpnav_vel = Twist()
        self.cmd_vel = Twist()
        self.has_obs = Bool()
        self.has_red_light = Bool()
        self.has_sign = Bool()
        self.sign_count = 0
       
        rospy.init_node('velocity_decision', anonymous=True)
        rospy.Subscriber('lane_vel', Twist, self.lanecallback)
        rospy.Subscriber('rp_nav', Twist, self.rpnavcallback)
        rospy.Subscriber('has_obs', Bool, self.obscallback)
        rospy.Subscriber('has_red_light', Bool, self.lightcallback)
        rospy.Subscriber('has_sign', Bool, self.signcallback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
    
    def lanecallback(self, msg):
        self.lane_vel = msg

    def rpnavcallback(self, msg):
        self.rpnav_vel = msg

    def obscallback(self, msg):
        self.has_obs = msg

    def lightcallback(self, msg):
        self.has_red_light = msg

    def signcallback(self, msg):
        self.has_sign = msg

    def spin(self):
        while not rospy.is_shutdown():
            if self.sign_count < 5:
                self.cmd_vel = self.lane_vel
                if self.has_obs.data == True:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = 0
                if self.has_red_light.data == True:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = 0
                if self.has_sign == True:
                    self.sign_count += 1
            else:
                self.cmd_vel = self.rpnav_vel
            self.pub.publish(self.cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        decide = velocityDecide()
        decide.spin()
    except rospy.ROSInterruptException:
        pass

    
        