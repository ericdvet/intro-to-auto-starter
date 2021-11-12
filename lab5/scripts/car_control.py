#!/usr/bin/env python
import rospy
from lab5.msg import SteeringInput
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import math

class CarControl:
    def __init__(self):
        rospy.init_node("car_control", anonymous=False)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/gap_error', SteeringInput, self.run_car)
        rospy.on_shutdown(self.on_shutdown)

    def run_car(self, steering_input):
        drive = AckermannDriveStamped()
        drive.drive.steering_angle = steering_input.steering_angle
        drive.drive.speed = 2
        self.drive_pub.publish(drive)
        pass

    def on_shutdown(self):
        print("Shutting down...")
        drive = AckermannDriveStamped()
        self.drive_pub.publish(drive)
        self.pose_pub.publish(PoseStamped())

if __name__ == "__main__":
    print("Starting Follow the Gap algorithm")
    CarControl()
    rospy.spin()
