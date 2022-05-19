#!/usr/bin/env python3

from ast import NameConstant
from turtle import color, forward, update
import rospy

import os
import numpy as np
import math
import cv2, cv_bridge

from std_msgs.msg import Bool, Header, String
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose, Point, PoseArray, PoseStamped, Quaternion

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

class Prediction(object):
    def __init__(self):
        rospy.init_node("prediction")

        self.initialized = False

        # --- INITIALIZE VISION ---
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # Subscribe to LIDAR
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # --- INITIALIZE MOVEMENT ---
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        # --- INITIALIZE ODOMOETRY ---
        self.base_frame = "base_footprint"
        self.odom_frame = "odom"

        self.current_pose = Pose()

        self.odom_pose_last_motion_update = None

        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # --- INITIALIZE TRACKING ---

        self.runner_history = []

        self.target_position = []

        rospy.sleep(3)

        self.initialized = True

    def image_callback(self, img):
        return

    def scan_callback(self, data):

        # --- UPDATE BASED ON ODOMETRY --- 
        # wait until initialization is complete
        if not(self.initialized):
            print("Not initialized")
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            print("Cannot transform?")
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            print("Waiting for transform")
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))


        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)
        print(self.laser_pose)
        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        self.update_current_pose()
        return

    def update_current_pose(self):
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        
        
        #Might need to invert calculations for yaw or something? DOUBLE CHECK
        delta_x = curr_x - old_x
        delta_y = curr_y - old_y

        rotation_1 = math.atan2(delta_y, delta_x) - old_yaw

        translation = math.sqrt( (delta_x * delta_x) + (delta_y * delta_y) )

        rotation_2 = curr_yaw - old_yaw - rotation_1

        #self.current_pose.position.x = 
        old_yaw = get_yaw_from_pose(self.current_pose)
        self.current_pose.position.x = self.current_pose.position.x + translation * math.cos(old_yaw + rotation_1)
        self.current_pose.position.y = self.current_pose.position.y + translation * math.sin(old_yaw + rotation_1)
        quat = quaternion_from_euler(0, 0, old_yaw + rotation_1 + rotation_2)
        self.current_pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        print(self.current_pose)
        return

    def add_tracking_point(self, angle, distance):
        #new_x = self.current_position.x

        return


if __name__ == "__main__":
    node = Prediction()

    rospy.spin()