#!/usr/bin/env python3

import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import datetime


class Camera(object):

    def __init__(self):

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.last_image = None
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        self.coordinates = []

    
    def image_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.last_image = image
        cv2.imshow("window", image)
        cv2.waitKey(3)
        pass


    def find_tag(self):

        
        img = self.last_image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        command = Twist()

        h, w, d = img.shape

        corners, ids, rejected_points = cv2.aruco.detectMarkers(gray, 
            self.aruco_dict)

        if corners is not None:
            


            #TODO: need to find the front of the robot and go in that direction to hit it. 
            coords = corners[0][0]    
            center = np.sum(coords, axis=0)[0] / 4
            self.coordinates.append( (center, datetime.datetime.now()))


            #cv2.circle(img, ())
            #command.angular.z = (-center + (w/2)) * 0.001






