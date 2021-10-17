#!/usr/bin/env python
import rospy

# TODO: import ROS msg types and libraries
from std_msgs.msg import String, Float32, Bool
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from math import cos, pi

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
        #self.ttc = []
        self.brake = Bool()
        pass

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ttc = []
        currentAngle = scan_msg.angle_min
        for i, j in enumerate(scan_msg.ranges):
            currentAngle += scan_msg.angle_increment
            ttci = 0
            if j > scan_msg.range_min and j < scan_msg.range_max and j != np.nan and j != np.inf:
                if (max(-1 * self.speed * cos(currentAngle), 0)) == 0:
                    ttci = 0
                else:
                    ttci = j / (max(-1 * self.speed * cos(currentAngle), 0) )
            ttc.append(ttci)
        #print(ttc[0], ttc[1])

        for i in ttc:
            if i < 1:
                brake = Bool()
                self.brake = True

        

        # TODO: publish brake message and publish controller bool

    def publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_brake_bool.publish(self.brake)
            rate.sleep()

        pass


if __name__ == '__main__':
    rospy.init_node('safety_node')
    safety_node = Safety()
    """try:
        print("Starting the Safety Node!")
        safety_node.publish()
        pass
    except rospy.ROSInterruptException:
        pass"""
    rospy.spin()