#!/usr/bin/env python3

import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import datetime
from sensor_msgs.msg import LaserScan


class Chase(object):

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # initalize the debugging window
        #cv2.namedWindow("window", 1)

        rospy.Subscriber("/scan", LaserScan, self.process_scan, queue_size =1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.coordinates = []
        self.artag_side = "none" #left, rightm or none


    def process_scan(self, data):
        
        
        if self.artag_side == "left":
            min_distance = 10000
            min_angle = 10000
            for i in np.arange(309, 360, 1):
                if data.ranges[i] < min_distance and data.ranges[i] != 0.0:
                    min_distance =  data.ranges[i]
                    min_angle = i

            min_angle = min_angle - 360

            print("ar tag is at", self.artag_side)
            print("angle", min_angle)
            print("distance",min_distance)
        elif self.artag_side == "right":
            min_distance = 10000 
            min_angle = 10000
            for i in np.arange(0, 52, 1):
                if  data.ranges[i] < min_distance and data.ranges[i] != 0.0:
                    min_distance =  data.ranges[i]
                    min_angle = i

            print("ar tag is at", self.artag_side)
            print("angle", min_angle)
            print("distance",min_distance)

        else:

            # tag is not found in camera
            pass

        

        


        return
    
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
                if ids[i][0] == 2: #here is the number of AR tag
                    for j in range(4):
                        tot_x += corners[i][0][j][0] #x
                        tot_y += corners[i][0][j][1] #y
                    cx = int(tot_x / 4)
                    cy = int(tot_y / 4)


                    self.coordinates.append(  (cx, cy, datetime.datetime.now()) )
                    tag_found = True
                
        # if there are any yellow pixels found
        if tag_found:

            # a red circle is visualized in the debugging window to indicate
            # the center point of the yellow pixels
            # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
            cv2.circle(image, (cx, cy), 10, (0,0,255), -1)
            if (w/2 - cx) > 0: 
                #left side
                self.artag_side = "left"
            else: 
                self.artag_side = "right"
        else:
            self.artag_side = "none"



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



