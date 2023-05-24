#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point

class TakePhoto:
    def __init__(self):

        # Initialize
        rospy.init_node('take_photo')

        self.count = 0
        self.bridge = CvBridge()
        self.image_received = False
        self.pose = Point()
        self.location = []

        # Connect image topic
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        rospy.Subscriber('obstacle_photo', UInt8, self.callback_obstacle)
        rospy.Subscriber('process/pose', Point, self.callback_pose)
        rospy.spin()


    def callback_obstacle(self, data):
        # Detects When Photo Needs to Be Taken from Obstacle Avoidance Algorithm
        location_dif =  True

        # Set Photo Title As x-y Location
        # Use '_image_title' parameter from command line
        # Default value is 'photo.jpg'
        title = 'photo_%0.3f, %0.3f.jpg' % (self.pose.x, self.pose.y)
        img_title = rospy.get_param('~image_title', title)

        # Check if Any Photos Were Taken in the Same Location
        for x in self.location:
            if abs(self.pose.x - x[0]) < 0.1 and abs(self.pose.y - x[1]) < 0.1:
                location_dif =  False
        
        # If Location is New, Take Picture
        if location_dif:
            self.location.append([self.pose.x, self.pose.y])
            pass
            if self.take_picture(img_title):
                rospy.loginfo("Saved image " + img_title)
                
                self.count += 1
            else:
                rospy.loginfo("No images received")


    def callback_pose(self, data):
        # Position Data
        self.pose = data


    def callback(self, data):
        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image


    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False


if __name__ == '__main__':
    TakePhoto()