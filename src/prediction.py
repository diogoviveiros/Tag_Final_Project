#!/usr/bin/env python3

from ast import NameConstant
from turtle import color, forward, update
import rospy
import numpy as np
import os
import cv2, cv_bridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist, Vector3

class Prediction(object):
    def __init__(self):
        rospy.init_node("prediction")

        # --- Initialize Vision ---
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # Subscribe to LIDAR
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # --- Initialize Movement ---
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        # --- Initialize Tracking Data
        self.runner_history = []

        rospy.sleep(3)

    def image_callback(self, img):
        return

    def scan_callback(self, data):
        return


if __name__ == "__main__":
    node = Prediction()

    rospy.spin()