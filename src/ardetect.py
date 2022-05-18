#!/usr/bin/env python3

import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import datetime


class Chase(object):

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # initalize the debugging window
        #cv2.namedWindow("window", 1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.coordinates = []

    
    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        h, w, d = image.shape
        # search for tags from DICT_4X4_50 in a GRAYSCALE image
        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)
        
        my_twist = Twist()

        tot_x = 0
        tot_y = 0
        cx = 0
        cy = 0
        tag_found = False
        if ids is not None:
            for i in range(len(ids)):
                if ids[i][0] == 2:
                    for j in range(4):
                        tot_x += corners[i][0][j][0] #x
                        tot_y += corners[i][0][j][1] #y
                    cx = int(tot_x / 4)
                    cy = int(tot_y / 4)
                    tag_found = True
                
        # if there are any yellow pixels found
        if tag_found:

            # a red circle is visualized in the debugging window to indicate
            # the center point of the yellow pixels
            # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
            cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

            # TODO: based on the location of the line (approximated
            #       by the center of the yellow pixels), implement
            #       proportional control to have the robot follow
            #       the yellow line
            k_a = 0.5
            k_l = -0.1
            e_a = (w/2 - cx)/w
            # if (abs(e_a) > 0.01):
            #     my_twist.angular.z = k_a * e_a
            # else:
            #     my_twist.angular.z = 0
            
            # if self.front_dist == 0:
            #     my_twist.linear.x = 0.1
            # else:
            #     dx = abs(self.distance - self.front_dist)
            #     if dx < self.buffer:
            #         my_twist.linear.x = 0
            #     else:
            #         e_l = (self.distance - self.front_dist)/self.front_dist
            #         my_twist.linear.x = k_l * e_l
        
        else:
            pass
            #my_twist.angular.z = 0.5

        self.cmd_pub.publish(my_twist)

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def run(self):
        rospy.spin()
                
if __name__ == '__main__':

        rospy.init_node('chaser')
        chasing = Chase()
        chasing.run()



