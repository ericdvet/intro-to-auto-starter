#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from lab4-launch-sample.msg import PIDInput

class Perception(object):

    def __init__(self):

        self.pub_input = rospy.Publisher("toit", PIDInput, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        pass

    def scan_callback(self, data):

        theta = 10
        a = data.ranges[int((theta+45) * (len(data.ranges)/270.0))]
        b = data.ranges[int(45.0 * (len(data.ranges)/270.0))]
        angle = np.arctan( (a * np.cos(np.deg2rad(theta)) - b) / (a * np.sin(np.deg2rad(theta))) )
        AB =  b * np.cos(np.deg2rad(angle))
        #print(AB)

        input = PIDInput()
        input = AB
        #self.pub_input.publish(input)

        pass

if __name__ == '__main__':
    rospy.init_node('perception_node')
    perception_node = Perception()
    rospy.spin()

