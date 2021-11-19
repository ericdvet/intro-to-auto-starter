#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from lab5.msg import SteeringInput
import numpy as np

class GapFinder:
    def __init__(self):
        rospy.init_node('gap_finder', anonymous=True)
        self.error_pub = rospy.Publisher('/gap_finder', SteeringInput, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, data):

        safeRanges = np.array(data.ranges[135:675])
        
        closesti = np.argmin(safeRanges)
        closestDist = np.amin(safeRanges)
        n = 90

        nn = int(n / closestDist)
        for i in range(closesti - nn, closesti + nn):
            if i >= 0 and i < len(safeRanges):
                safeRanges[i] = 0

        farthesti = np.argmax(safeRanges)
        farthestAngle = farthesti * data.angle_increment - math.pi / 2.0

        self.error_pub.publish(farthestAngle)

if __name__ == "__main__":
    GapFinder()
    rospy.spin()
