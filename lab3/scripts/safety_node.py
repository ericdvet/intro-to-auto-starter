#!/usr/bin/env python
import rospy

# TODO: import ROS msg types and libraries
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from math import cos

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        self.pub_brake = rospy.Publisher("brake", AckermannDriveStamped, queue_size=10)
        self.pub_brake_bool = rospy.Publisher("brake_bool", Bool, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.brake = AckermannDriveStamped()
        self.brake_bool = Bool()
        pass

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        if self.speed > 0.01    :
            minRange = 1000.0
            angle = scan_msg.angle_min
            for i in scan_msg.ranges:
                if angle >= scan_msg.angle_min / 2.0:
                    if i > scan_msg.range_min and i < scan_msg.range_max and i != np.nan and i != np.inf:
                        if i < minRange:
                            minRange = i
                            minAngle = angle
                angle += scan_msg.angle_increment 
                if angle >= scan_msg.angle_max  / 2.0:
                    break
            tts = minRange / (self.speed * cos(minAngle))
            #print(minRange, minAngle, tts)
            self.brake_bool.data = False
            if tts < 0.5:
                self.brake_bool.data = True
                self.brake.drive.speed = 0
                self.pub_brake.publish(self.brake)
                self.pub_brake_bool.publish(self.brake_bool)
                self.brake_bool.data = False

        pass


if __name__ == '__main__':
    rospy.init_node('safety_node')
    safety_node = Safety()
    rospy.spin() 