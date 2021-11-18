#!/usr/bin/env python
import rospy
from lab5.msg import SteeringInput
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import math

ERROR_SCALE = 1
MAX_STEERING_ANGLE = 4.18

class CarControl:
    def __init__(self):
        rospy.init_node("car_control", anonymous=False)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/gap_error', SteeringInput, self.run_car)
        rospy.on_shutdown(self.on_shutdown)
        self.kP = 1

    def run_car(self, steering_input):
        SCALED_ERROR = steering_input.steering_angle * ERROR_SCALE
        steering_angle = self.kP * SCALED_ERROR

        if (steering_angle > MAX_STEERING_ANGLE):
            steering_angle = MAX_STEERING_ANGLE
        if (steering_angle < (-1 * MAX_STEERING_ANGLE)):
            steering_angle = -1 * MAX_STEERING_ANGLE

        drive = AckermannDriveStamped()
        drive.drive.steering_angle = steering_angle
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
