#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import math
import numpy


class sensors():
        
    def __init__(self):
        # Class Variables
        self.bump = BumperEvent()
        self.scan = Point()
        self.scan_prev = Point()
        self.pose = Point()
        self.yaw = 0

        # Node Initialisation
        rospy.init_node('Sensors')

        # Publishers
        self.pub_scan = rospy.Publisher('process/scan', Point, queue_size=10)
        self.pub_pose = rospy.Publisher('process/pose', Point, queue_size=10)
        self.pub_yaw = rospy.Publisher('process/yaw', Float64, queue_size=10)
        self.pub_bump = rospy.Publisher('process/bump', BumperEvent, queue_size=10)

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.callback_scan) 
        rospy.Subscriber("/odom", Odometry, self.callback_odom)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.callback_bump)

        # Set Refresh Rate
        r = rospy.Rate(10)

        # Updating Loop
        while not rospy.is_shutdown():
            self.pub_scan.publish(self.scan)
            self.pub_pose.publish(self.pose)
            self.pub_yaw.publish(self.yaw)
            self.pub_bump.publish(self.bump)
            r.sleep()

    
    def callback_scan(self, data):
        # Split Laser Signal into 5 Zones with Max Min and Average Readings
        Right = self.nan_conversion(min(data.ranges[0:256]))
        Centre = self.nan_conversion(min(data.ranges[257:384]))
        Left = self.nan_conversion(min(data.ranges[385:640]))

        # Check if the robot is close to the wall when the sensor errors out
        if Left == 10 and self.scan_prev.x < 0.5:
            Left = 0.45
        if Centre == 10 and self.scan_prev.y < 0.5:
            Centre = 0.45
        if Right == 10 and self.scan_prev.z < 0.5:
            Right = 0.45

        # Publish Results
        self.scan.x = Left
        self.scan.y = Centre
        self.scan.z = Right
        self.scan_prev = self.scan


    def callback_odom(self, data):
        # Get Data for Robot Data and Pose
        self.pose = data.pose.pose.position

        # Convert fron Quaternion to Euler for Yaw
        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw
    

    def callback_bump(self, data):
        # Bumper Data
        self.bump = data


    def nan_conversion(self, data):
        # When Laser Reads nan Convert to 10 Metres
        if math.isnan(data):
            data = 10
        return data


if __name__ == '__main__':
    sensors()