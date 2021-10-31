#!/usr/bin/env python
import rospy
from lab4.msg import PIDInput
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import math

ERROR_SCALE = 1 # TODO: ASSIGN A CONSTANT FOR HOW MUCH TO SCALE THE RAW ERROR
MAX_STEERING_ANGLE = 4.18

class CarControl:
    def __init__(self, kP, kD):
        # initialize ros node
        rospy.init_node("car_control", anonymous=False)

        # get publisher handles for drive and pose topics 
        # TODO: CREATE A PUBLISHER FOR THE DRIVE TOPIC 
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        # pose topic is to reset car position when you reset/shutdown the node
        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)

        # TODO: SUBSCRIBE TO dist_error MESSAGE FROM dist_finder.py NODE . ASSIGN THE CALLBACK TO run_pid()
        self.previousError = 0
        rospy.Subscriber('/dist_error', PIDInput, self.run_pid)

        # TODO: INITIALIZE KP AND KD CLASS VARIABLES FROM CONSTRUCTOR PARAMETERS
        self.kP = 1
        self.kD = 0.6


        # assign shutdown functon 
        rospy.on_shutdown(self.on_shutdown)

    # callback for dist_error topic to handle pid calculation and publish to /drive. 
    def run_pid(self, pid_input):
        # TODO: SCALE ERROR BY SOME CONSTANT
        SCALED_ERROR = pid_input.angle_error * ERROR_SCALE
        dERROR = (self.previousError - SCALED_ERROR)

        # TODO: CALCULATE A STEERING ANGLE BASED BY PASSING THE ERROR INTO THE PID FUNCTION
        steering_angle = (self.kP * SCALED_ERROR) + (self.kD * -1 * (self.previousError - SCALED_ERROR))
        
        # TODO: UPDATE PREVIOUS ERROR FOR DERIVATIVE TERM
        self.previousError = SCALED_ERROR

        # TODO: VALIDATE STEERING ANGLE
        if (steering_angle > MAX_STEERING_ANGLE):
            steering_angle = MAX_STEERING_ANGLE
        if (steering_angle < (-1 * MAX_STEERING_ANGLE)):
            steering_angle = -1 * MAX_STEERING_ANGLE

        # TODO: ASSIGN VALUES IN ACKERMANN MESSAGE 
        drive = AckermannDriveStamped()
        drive.drive.steering_angle = steering_angle
        if (abs(pid_input.angle_error) < 0.3):
            drive.drive.speed = 5
        elif (abs(pid_input.angle_error) < 0.6):
            drive.drive.speed = 3
        else:
            drive.drive.speed = 2

        #print(SCALED_ERROR, self.previousError, dERROR)
    
        # TODO PUBLISH ACKERMANN DRIVE MESSAGE
        self.drive_pub.publish(drive)

        # OPTIONAL: TO IMPROVE YOUR WALL FOLLOWING TRY ADDING ADAPTIVE VELOCITY CONTROL

        pass


    # shutdown function
    def on_shutdown(self):
        print("### END WALL FOLLOW ###")

        # TODO PUBLISH AN EMPTY ACKERMANNDRIVESTAMPED MESSAGE TO THE DRIVE PUBLISHER HANDLE
        drive = AckermannDriveStamped()
        self.drive_pub.publish(drive)

        self.pose_pub.publish(PoseStamped())


if __name__ == "__main__":
    #  TODO: USE USER INPUT TO TUNE KP AND KD, THEN HARDCODE ONCE TUNED
    kP = 1#float(input("Enter kP: "))
    kD = 1#float(input("Enter kD: "))

    print("### START WALL FOLLOW ###")

    # initial node
    CarControl(kP, kD)

    # spin ros
    rospy.spin()
