#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from lab4.msg import PIDInput

# helpful constants:
ANGLE_RANGE_DEG = 270
ANGLE_RANGE_RAD = 4.72
CAR_LEN = 1.5


DESIRED_DIST_FROM_WALL = 0.5 # TODO: SET A DESIRED DISTANCE AWAY FROM THE WALL

WF_ZERO  = 45   # TODO: DEFINE WHAT YOUR "ZERO" ANGLE WILL BE TO FIND THE RANGE PERPENDICULAR TO THE CAR'S FORWARD VECTOR
WF_THETA = 55   # TODO: SET THETA VALUE (HINT: SHOULD BE > WF_ZERO)


class DistFinder:
    def __init__(self, wall_dist=DESIRED_DIST_FROM_WALL):
        # set pid control vars
        self.dist_desired = wall_dist

        # initialize node to register under master
        rospy.init_node('dist_finder', anonymous=True)


        # TODO: CREATE AN PUBLISHER HANDLE TO PUBLISH TO THE /dist_error TOPIC
        self.error_pub = rospy.Publisher('/dist_error', PIDInput, queue_size=10)

        # subscribe to scan topic of simulator
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
    

    # get_range converts a theta value in degrees to a range in the ranges array
    def get_range(self, data, theta):
        # TODO: USE THE MAPPING IN THE PRELAB OR A MAPPING OF YOUR CHOOSING TO 
        #       CONVERT FROM A THETA TO A RANGE IN THE DATA
        return data.ranges[int(theta * (len(data.ranges) / 270.0))]

    def scan_callback(self, data):
        
        # TODO CALCULATE DISTANCE AWAY FROM THE WALL USING FORMULA SEEN IN PRELAB
        # B IS GIVEN AS AN EXAMPLE, AND YOU NEED TO FILL IN THE REST
        a = self.get_range(data, WF_THETA)
        b = self.get_range(data, WF_ZERO)	


        # math functions use radians, so use math.radians to convert for the alpha calculation
        theta_rad = math.radians(WF_THETA)

        ## Your code goes here to compute alpha, AB, and CD..and finally the error.
        # TODO: COMPUTE ALPHA, AB, CD AND THEN THE ERROR
        alpha = math.atan( (a * math.cos(theta_rad) - b) / (a * math.sin(theta_rad)) )
        AB = b * math.cos(alpha)
        AC = CAR_LEN
        CD = AB + (AC * math.sin(alpha))
        ERROR = DESIRED_DIST_FROM_WALL - CD


        # TODO: CREATE AN INSTANCE OF THE PIDInput() MESSAGE AND ASSIGN THE ANGLE ERROR
        error = PIDInput()
        error.angle_error = ERROR
        #print(CD)

        # TODO: PUBLISH YOUR MESSAGE USING self.error_pub
        self.error_pub.publish(error)
        



if __name__ == "__main__":
    # OPTIONAL: ADD USER INPUT TO DEFINE DESIRED DISTANCE FROM WALL
    DistFinder()
    
    rospy.spin()
