#!/usr/bin/env python
import rospy
from lab5.msg import SteeringInput
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import math

ERROR_SCALE = 1 # TODO: ASSIGN A CONSTANT FOR HOW MUCH TO SCALE THE RAW ERROR
MAX_STEERING_ANGLE = 2.71

"""class CarControl:
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
        rospy.Subscriber('/gap_finder', SteeringInput, self.run_pid)

        # TODO: INITIALIZE KP AND KD CLASS VARIABLES FROM CONSTRUCTOR PARAMETERS
        self.kP = 1.2
        self.kD = -5


        # assign shutdown functon 
        rospy.on_shutdown(self.on_shutdown)

    # callback for dist_error topic to handle pid calculation and publish to /drive. 
    def run_pid(self, pid_input):
        # TODO: SCALE ERROR BY SOME CONSTANT
        SCALED_ERROR = pid_input.steering_angle * ERROR_SCALE
        dERROR = (self.previousError - SCALED_ERROR)

        # TODO: CALCULATE A STEERING ANGLE BASED BY PASSING THE ERROR INTO THE PID FUNCTION
        steering_angle = (self.kP * SCALED_ERROR)# + (self.kD * (self.previousError - SCALED_ERROR))
        #print(steering_angle, SCALED_ERROR, dERROR)
        
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
        drive.drive.speed = 1

        print(steering_angle, SCALED_ERROR, dERROR)
    
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


"""
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
        rospy.Subscriber('/gap_finder', SteeringInput, self.run_car)
        rospy.on_shutdown(self.on_shutdown)

    def run_car(self, steering_input):
        
        steering_A = steering_input.steering_angle

        #print(steering_A)

        drive = AckermannDriveStamped()
        drive.drive.steering_angle = steering_A
        """if (steering_A < 0.1):
            drive.drive.speed = 5
        elif (steering_A < 1):
            drive.drive.speed = 4
        else:
            drive.drive.speed = 2"""
        drive.drive.speed = 2.5

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
