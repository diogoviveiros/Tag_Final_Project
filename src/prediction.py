#!/usr/bin/env python3

from ast import NameConstant
from hashlib import new
from turtle import color, forward, update
import rospy

import os
import numpy as np
import math
from sklearn.linear_model import LinearRegression
import cv2, cv_bridge
from datetime import datetime

from std_msgs.msg import Bool, Header, String
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose, Point, PoseArray, PoseStamped, Quaternion
from tag_final_project.msg import AngleVector

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
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        self.curr_pose = Pose()

        self.odom_pose_last_motion_update = None

        self.odom_pose = None

        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # --- INITIALIZE TRACKING ---
        self.array_size = 100
        self.runner_points = []
        self.runner_times = []
        self.curr_distance = 0

        # self.runner_history = []
        # self.target_position = []

        # publish the current runner history
        self.path_pub = rospy.Publisher("path_poses", PoseArray, queue_size=10)

        # Add angle vector subscription
        self.angle_vec_sub = rospy.Subscriber('angle_vectors', AngleVector, self.vector_callback)

        # TODO: subscription to bump topic
        # self.bumped_sub = rospy.Subscriber('', , self.bumped_callback)
        self.bumped = False

        rospy.sleep(3)

        self.initialized = True


    def scan_callback(self, data):
        # gets the current pose of chaser robot using odometry

        # --- UPDATE BASED ON ODOMETRY --- 
        # wait until initialization is complete
        if not(self.initialized):
            print("Pred: Not initialized")
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            print("Pred: Cannot transform?")
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            print("Pred: Waiting for transform")
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

        print(self.odom_pose)

        self.curr_pose = self.odom_pose.pose

        print(self.curr_pose)
        return

    def vector_callback(self, data):
        # callback function upon receiving information about runner
        # processes info to get position and time history of runner 
        print("Pred: Recieved Angle Vector (" + str(data.angle) + "," + str(data.distance) + ")")
        self.add_tracking_point(data.angle, data.distance)
        self.publish_runner_history()
        return

    def add_tracking_point(self, angle, distance):
        # calculates absolute position of runner on odometry map 
        curr_angle = get_yaw_from_pose(self.curr_pose)
        x = self.curr_pose.position.x + math.cos(angle + curr_angle) * distance
        y = self.curr_pose.position.y + math.sin(angle + curr_angle) * distance

        # store current distance to runner
        self.curr_distance = distance
        
        # store absolute path of runner and time info
        target = Point()
        target.x = x
        target.y = y

        self.runner_points.append(target)
        self.runner_times.append(datetime.now())

        # pop oldest history if greater than array length
        if len(self.runner_points) > self.array_size:
            self.runner_points.pop()
            self.runner_times.pop()

        # self.runner_history.append((target_pose, datetime.now()))
        return

    def publish_runner_history(self):
        # publishes runner history (IDK if necessary?)
        history_pose_array = PoseArray()
        history_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        history_pose_array.poses
    
        for point in self.runner_history:
            (point_pose, point_time) = point
            history_pose_array.poses.append(point_pose)

        print("Pred: Publishing particle cloud of size: " + str(len(self.runner_history)))

        self.path_pub.publish(history_pose_array)

    def bump_callback(self, data):
        # TODO: stop chasing if bumped into target
        self.bumped = data

    def predict(self):
        # continuous loop that publishes vectors to move chaser in predicted path of runner
        r = rospy.Rate(0.5)

        while not rospy.is_shutdown() and not self.bumped:
            # wait until we have filled runner history
            if len(self.runner_points) == self.array_size:
                # get x and y paths separately
                xs = []
                ys = []

                for p in self.runner_points:
                    xs.append(p.x)
                    ys.append(p.y)

                # convert to arrays
                xs = np.array(xs)
                ys = np.array(ys)
                ts = np.array(self.runner_times)
                ts = ts - ts[0] # make first timepoint 0

                # predict next position of chaser proportionally to distance
                v = 0.2
                delta = 0.5 # basically how much more velocity to cover current distance
                pred_time = self.curr_distance / (v + delta) # how far into future we want to predict, should be < set velocity

                # linear regression of x vs. t and y vs. t
                # can try polynomial regression? 
                modelx = LinearRegression().fit(xs,ts)
                modely = LinearRegression().fit(ys,ts)

                print(f"x slope: {modelx.coef_}")
                print(f"y slope: {modely.coef_}")

                pred_x = modelx.coef_ * pred_time + modelx.intercept
                pred_y = modely.coef_ * pred_time + modely.intercept

                # calculate proportional turn towards predicted point
                # linear velocity proportional to distance but has ceiling v
                # implement derivative or integral? I dont think so since we probably aren't moving fast
                dx = pred_x - self.curr_pose.position.x
                dy = pred_y - self.curr_pose.position.y
                
                pred_theta = math.atan2(dy, dx)
                pred_dist = math.sqrt(dx**2 + dy**2)
                ka = 0.005
                kl = 0.2

                twist = Twist()
                twist.angular.z = ka * pred_theta
                twist.linear.x = max(kl * pred_dist, v)
                self.twist_pub(twist)

            r.sleep()
        
        rospy.spin()



if __name__ == "__main__":
    node = Prediction()
    node.predict()
    