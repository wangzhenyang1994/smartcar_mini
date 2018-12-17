#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

class rplidarDetector:
    def __init__(self):
        self.i = 0
        self.has_obs = Bool()

        rospy.init_node('obstacle_detection', anonymous=True)
        self.pub = rospy.Publisher('has_obs', Bool, queue_size=1)
        self.rate = rospy.Rate(10)

    def spin(self):
        while not rospy.is_shutdown():
            if self.i < 100:
                self.has_obs.data = False
                self.pub.publish(self.has_obs)
            else:
                self.has_obs.data = True
                self.pub.publish(self.has_obs)
            self.i +=1
            if self.i > 100:
                self.i %= 100
            self.rate.sleep()

if __name__ == '__main__':
    try:
        detector = rplidarDetector()
        detector.spin()
    except rospy.ROSInterruptException:
        pass
