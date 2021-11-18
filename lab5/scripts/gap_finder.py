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

        safetyRanges = list(data.ranges)
        closestDist = data.ranges[0]
        closesti = 0
        for (i, dist) in enumerate(safetyRanges):
            if dist > data.range_min and dist < data.range_max and dist != np.nan and dist != np.inf:
                if dist < closestDist:
                    closestDist = dist
                    closesti = i
        
        bubbleSize = 10
        for i in range(closesti - int(closestDist*bubbleSize), closesti + int(closestDist*bubbleSize) ):
            if i > 0 and i < len(safetyRanges):
                safetyRanges[i] = 0
        
        farthestDist = data.ranges[0]
        farthesti = 0
        farthestAngle = data.angle_min
        angle = data.angle_min
        for (i, dist) in enumerate(safetyRanges):
            if dist > data.range_min and dist < data.range_max and dist != np.nan and dist != np.inf:
                if dist > farthestDist:
                    farthestDist = dist
                    farthesti = i
                    farthestAngle = angle
            angle += data.angle_increment
        print(farthestAngle, farthestDist, farthesti)

        self.error_pub.publish(farthestAngle)

if __name__ == "__main__":
    GapFinder()
    rospy.spin()
