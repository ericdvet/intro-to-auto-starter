#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from lab5.msg import SteeringInput
import numpy as np

class GapFinder:
    def __init__(self):
        rospy.init_node('gap_finder', anonymous=True)
        self.error_pub = rospy.Publisher('/gap_error', SteeringInput, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
    
    def get_range(self, data, theta):
        return data.ranges[int(theta * (len(data.ranges) / 270.0))]

    def scan_callback(self, data):
        farthestDist = data.ranges[0]
        farthestAngle = data.angle_min
        anglei = 0
        for (i, dist) in enumerate(data.ranges):
            if dist > data.range_min and dist < data.range_max and dist != np.nan and dist != np.inf:
                if (dist > farthestDist):
                    farthestDist = dist
                    farthestAngle = anglei
            anglei += data.angle_increment
        self.error_pub.publish(farthestAngle - (math.pi / 4.0) - (math.pi / 2.0))

if __name__ == "__main__":
    GapFinder()
    rospy.spin()
