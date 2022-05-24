#!/usr/bin/env python3

import rospy
import numpy as np
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import datetime
from sensor_msgs.msg import LaserScan
from tag_final_project.msg import AngleVector

class DetectRunner(object):

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.cam_angle = 0
        self.object_distance = 0

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.process_scan, queue_size =1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.coordinates = []
        self.artag_side = "none" #left, right, or none

        # publisher for angle and distance of runner robot
        self.angle_vec_pub = rospy.Publisher('angle_vectors', AngleVector, queue_size=10)
        

    def process_scan(self, data):
        # get runner position as angle and min dist from laserscan
        rs = np.nan_to_num(data.ranges)

        # get min distance validated by which side of the camera the marker is detected on
        if self.artag_side == "left":
            rs = rs[0:52]
            self.object_distance = np.amin(rs[np.nonzero(rs)])
            self.cam_angle = np.argmin(rs[np.nonzero(rs)])

            print("AR: ar tag is at", self.artag_side)
            print("AR: angle", self.cam_angle)
            print("AR: distance", self.object_distance)
        
        elif self.artag_side == "right":
            rs = rs[309:359]
            self.object_distance = np.amin(rs[np.nonzero(rs)])
            self.cam_angle = 360 - np.argmin(rs[np.nonzero(rs)])

            print("AR: ar tag is at", self.artag_side)
            print("AR: angle",self.cam_angle)
            print("AR: distance", self.object_distance)

        # tag not found
        else:
            pass

    
    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        _, w, _ = image.shape
        # search for tags from DICT_4X4_50 in a GRAYSCALE image
        corners, ids, _ = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

        # get centers of all detected markers
        cxs = []
        cys = []

        if ids is not None:
            for i in range(len(ids)):
                cxs.append(int((corners[i][0][0][0] + corners[i][0][2][0]) / 2))
                cys.append(int((corners[i][0][0][1] + corners[i][0][2][1]) / 2))

            # get the overall center of markers 
            cx = int(np.mean(cxs))
            cy = int(np.mean(cys))

            # a red circle is visualized in the debugging window
            cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

            # this might not be as robust if we are not detecting a subset of markers
            if (w/2 - cx) > 0: 
                #left side
                self.artag_side = "left"
            else: 
                self.artag_side = "right"
        else:
            self.artag_side = "none"

        my_angle_vec = AngleVector()
        my_angle_vec.angle = self.cam_angle 
        my_angle_vec.distance = self.object_distance 
        # i think we should publish time data here as well
        self.angle_vec_pub.publish(my_angle_vec)

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def run(self):
        rospy.spin()
                
if __name__ == '__main__':
        rospy.init_node('detect_runner')
        detector = DetectRunner()
        detector.run()



