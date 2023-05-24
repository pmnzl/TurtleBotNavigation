#! /usr/bin/env python

# Dependencies
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
import math
import numpy


class bug2():

    def __init__(self):
        # Node Initialisation
        rospy.init_node("bug2")

        # Topic Subscribers
        rospy.Subscriber('process/scan', Point, self.callback_laserscan)
        rospy.Subscriber('process/pose', Point, self.callback_pose)
        rospy.Subscriber('process/yaw', Float64, self.callback_yaw)
        rospy.Subscriber('process/bump', BumperEvent, self.callback_bumper)

        # Topic Publishers
        self.move = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.pic = rospy.Publisher('obstacle_photo', UInt8, queue_size=10)

        # Position Varibles
        self.pose_initial = Point()
        self.pose_goal = Point()
        self.pose = Point()
        self.yaw = 0
        self.bumper_pose = 0

        # Sensor/Actuator Variables
        self.bump = []
        self.bump_hit = False
        self.laser = []
        self.move_cmd = Twist()

        # Variable Initialisation
        self.line_distance = 0
        self.pose_goal.x = 20
        self.pose_goal.y = 0
        self.laser_aquired = False
        self.distance_threshold = 1.0
        self.pose_initial = self.pose
        self.count = 0
        self.block_bumper = False

        # States
        self.state_global = 0
        self.state_line = 0
        self.state_bumper = 0

        # Tolerances
        self.yaw_tolerance = math.pi/90
        self.distance_tolerance = 0.3

        # ROS Loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Calculate Goals for Path Planning
            self.yaw_goal = math.atan2(self.pose_goal.y - self.pose.y, self.pose_goal.x - self.pose.x)
            self.line_distance = self.calculate_line_distance() 

            # Only Continue if Laser Scanner Has Been Read
            if self.laser_aquired == True:

                if self.state_global == 0:  # Drive To Goal State
                    self.go_to_point()
                    if self.laser[1] > 0.45 and self.laser[1] < 0.6: # Exit Condition to Laser Avoidance State
                        self.pic.publish() # Take Picture
                        self.state_global = 1

                elif self.state_global == 1: # Laser Avoidance State
                    self.laser_avoid()
                    if self.count > 100 and self.line_distance < 0.1: # Laser Avoidance Exit Condition
                        self.count = 0
                        self.state_global = 0

                elif self.state_global == 2: # Bumper State
                    self.bumper_hit()
                    if self.laser[1] > 0.45 and self.laser[1] < 0.6: # Exit Condition to Laser Avoidance State
                        self.pic.publish() # Take Picture
                        self.state_global = 1
                   
                self.count += 1 # Count State Iterations

            # Publish Move Commands to the Robot
            self.move.publish(self.move_cmd)

            # Delay Loop
            rate.sleep()


    def callback_laserscan(self, data):
        # Split Laser Signal into 5 Zones with Max Min and Average Readings
        Left = data.x
        Centre = data.y
        Right = data.z
        self.laser = [Right, Centre, Left]
        self.laser_aquired = True


    def callback_pose(self, data):
        # Get Pose Data from Odometry
        self.pose = data


    def callback_yaw(self, data):
        # Get Yaw Data from Odometry
        self.yaw = data.data


    def callback_bumper(self, data):
        # Sends Back State and Bumper No.
        # Blocker used when bumper state is triggered by the laser scan
        if not self.block_bumper or data.state == 1:
            self.bump = data
            
        # Check if Bumper is Hit to Go to Bumper State
        if data.state == 1:
            self.bumper_pose = self.pose
            self.state_global = 2
            self.state_bumper = 0
        

    def go_to_point(self):
        # State Machine for Driving to Goal
        if self.state_line == 0: # When not Aligned to Goal
            self.yaw_correction()
        elif self.state_line == 1: # When Aligned to Goal
            self.go_straight()
        elif self.state_line == 2: # When at Goal
            self.stop()
        else:
            rospy.logerr("Not A State")


    def laser_avoid(self):
        # Get Logical Values of Laser Scan Zones
        Left_Reading = self.laser[2] < self.distance_threshold
        Centre_Reading = self.laser[1] < self.distance_threshold
        Right_Reading = self.laser[0] < self.distance_threshold
        Reading = [Left_Reading, Centre_Reading, Right_Reading]

        # Logical Values -> Movement
        if Reading == [0, 0, 0] or Reading == [1, 0, 1]:
            # Straight
            self.move_cmd.linear.x = 0.5
            self.move_cmd.angular.z = 0
        elif Reading == [0, 1, 0] or Reading == [1, 1, 1]:
            # Bumper Call
            self.block_bumper = True
            if self.laser[2] < self.laser[0]: # If Object Detected on Right, Turn Left
                self.bump.bumper = 0 
            else:                             # If Object Detected on Left, Turn Right
                self.bump.bumper = 2
            
            # Set Bumnper Parameters
            self.bumper_pose = self.pose
            self.state_global = 2
            self.state_bumper = 0

        elif Reading == [0, 0, 1]:
            # Straight Left
            self.move_cmd.linear.x = 0.5
            self.move_cmd.angular.z = 0.3

        elif Reading == [0, 1, 1]:
            # Left
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0.3

        elif Reading == [1, 0, 0]:
            # Straight Right
            self.move_cmd.linear.x = 0.5
            self.move_cmd.angular.z = -0.3

        elif Reading == [1, 1, 0]:
            # Right
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = -0.3

        else:
            rospy.logerr("State Does Not Exist")


    def bumper_hit(self):
        # States For When the Bumper Gets Hit
        if self.state_bumper == 0: # Robot Needs to Move Back
            self.bumper_back()
        elif self.state_bumper == 1: # Robot Needs to Rotate Clockwise
            self.bumper_rotate()
        elif self.state_bumper == 2: # Robot Needs to Drive Straight
            self.bumper_straight()
        else:
            rospy.logerr("Not A State")


    def bumper_back(self):
        # Move Back a Specified Distance
        self.stop()
        error_pose = math.sqrt(pow(self.bumper_pose.y - self.pose.y, 2) + pow(self.bumper_pose.x - self.pose.x, 2))
        self.move_cmd.linear.x = -0.4
        self.move_cmd.angular.z = 0

        if error_pose > 0.3:
            # Initialise Rotation State
            self.bumper_yaw = self.yaw
            self.pic.publish() # Take Picture
            self.state_bumper = 1
        

    def bumper_rotate(self):
        # Rotate 90 Degrees Clockwise
        # Calculate Yaw Error
        error_yaw = self.correct_angle(self.bumper_yaw - self.yaw) # 90 Deg in Rad
        self.move_cmd.linear.x = 0

        # Determine Left or Right Turn
        if self.bump.bumper == 0 or self.bump.bumper == 1:
            self.move_cmd.angular.z = -0.7
        else:
            self.move_cmd.angular.z = 0.7

        # Exit Condtion
        if abs(error_yaw) > 0.2*math.pi: # 90 Degrees Clockwise
            # Initialise Straight State
            self.bumper_pose = self.pose
            self.state_bumper = 2 


    def bumper_straight(self):
        # Move Foward a Specified Distance
        # Calculate Distance To Position
        error_pose = math.sqrt(pow(self.bumper_pose.y - self.pose.y, 2) + pow(self.bumper_pose.x - self.pose.x, 2))
        self.move_cmd.linear.x = 0.4
        self.move_cmd.angular.z = 0

        # Exit Condition
        if error_pose > 1.0:
            # Initialise Next Bump and Exit Bumper State Machine
            self.block_bumper = False
            self.state_bumper = 0
            self.state_global = 0


    def yaw_correction(self):
        # Corrects the Yaw of the Robot When off Course from the Goal
        error_yaw = self.correct_angle(self.yaw_goal - self.yaw)
        # Rotation Logic
        if math.fabs(error_yaw) > self.yaw_tolerance:
            self.move_cmd.angular.z = 0.4 if error_yaw > 0 else -0.4
            self.move_cmd.linear.x = 0
    
        # Exit Condtion
        if math.fabs(error_yaw) <= self.yaw_tolerance:
            self.stop()
            self.move_cmd.angular.z = 0
            self.state_line = 1


    def go_straight(self):
        # Drive Straight While Avoiding Obstacles
        # Calculate the Yaw and Pose Error
        error_yaw = self.correct_angle(self.yaw_goal - self.yaw)
        error_pose = math.sqrt(pow(self.pose_goal.y - self.pose.y, 2) + pow(self.pose_goal.x - self.pose.x, 2))
        # Movement
        if error_pose > self.distance_tolerance:
            self.move_cmd.linear.x = 0.6
            self.move_cmd.angular.z = 0
        else:
            self.stop()
            self.state_line = 2 # Exit Condition when Robot Has Reached the Goal
        
        # Exit Condition
        if math.fabs(error_yaw) > self.yaw_tolerance:
            self.stop()
            self.state_line = 0


    def stop(self):
        # Stop Robot Movement
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0


    def calculate_line_distance(self):
        # Calculates the Distance to the Goal
        num = math.fabs((self.pose_goal.y - self.pose_initial.y) * self.pose.x - (self.pose_goal.x - self.pose_initial.x) * self.pose.y + (self.pose_goal.x * self.pose_initial.y) - (self.pose_goal.y * self.pose_initial.x))
        den = math.sqrt(pow(self.pose_goal.y - self.pose_initial.y, 2) + pow(self.pose_goal.x - self.pose_initial.x, 2))
        distance = num / den

        return distance


    def correct_angle(self, angle):
        # Corrects for Roational Angle Logic
        if(math.fabs(angle) > math.pi):
            angle = angle - (2*math.pi*angle)/(math.fabs(angle))

        return angle


    def nan_conversion(self, data):
        # When Laser Reads nan Convert to 10 Metres
        if math.isnan(data):
            data = 10
            
        return data

    
if __name__ == '__main__':
    bug2()