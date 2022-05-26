#!/usr/bin/env python3

from ast import NameConstant
from hashlib import new
from turtle import color, forward, update
import rospy

import os
import numpy as np
import math
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import cv2, cv_bridge
from datetime import datetime

from std_msgs.msg import Bool, Header, String
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose, Point, PoseArray, PoseStamped, Quaternion
from turtlebot3_msgs.msg import SensorState
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
        self.bumped = False
        # --- INITIALIZE VISION ---
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Subscribe to LIDAR
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # --- INITIALIZE MOVEMENT ---
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        self.twist = Twist()

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
        self.array_size = 30 # number of historical points to store ~ 8s of history
        self.runner_points = []
        self.runner_times = []
        self.curr_distance = 0

        # publish the current runner history
        self.path_pub = rospy.Publisher("path_poses", PoseArray, queue_size=10)

        # Publish the targetted pose
        self.target_pose_pub = rospy.Publisher("predicted_pose", PoseStamped, queue_size=10)

        # Add angle vector subscription
        rospy.Subscriber('angle_vectors', AngleVector, self.vector_callback)

        # subscription to bumper topic
        rospy.Subscriber('sensor_state', SensorState, self.bumper_callback, queue_size = 10)
        

        # store prediction models
        self.modelx = None
        self.modely = None 

        rospy.sleep(5)

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

        #print(self.odom_pose)

        self.curr_pose = self.odom_pose.pose

        #print("curr_pose:",self.curr_pose)
        return


    def vector_callback(self, data):
        # callback function upon receiving information about runner
        # processes info to get position and time history of runner 
        #print("Pred: Received Angle Vector (" + str(data.angle) + "," + str(data.distance) + ")")
        if not self.initialized:
            return
        self.add_tracking_point(data.angle, data.distance, data.timestamp)
        self.publish_runner_history()


    def add_tracking_point(self, angle, distance, time):
        # calculates absolute position of runner on odometry map 
        
        if angle == -1 and distance == -1:
            # if ar tag is not detected, predict missing path using existing models
            if not self.modelx and not self.modely:
                return
            assert self.modelx, "Model X does not exist"
            assert self.modely, "Model Y does not exist"
            x, y = self.predict(time - self.runner_times[0])
        else:
            # calculate position of runner
            curr_angle = get_yaw_from_pose(self.curr_pose)
            angle *= np.pi / 180
            self.curr_distance = distance

            x = self.curr_pose.position.x + math.cos(angle + curr_angle) * distance
            y = self.curr_pose.position.y + math.sin(angle + curr_angle) * distance

        # store path of runner
        target = Point()
        target.x = x
        target.y = y
        self.runner_points.append(target)
        
        # store time info
        self.runner_times.append(time) 

        # pop oldest history if greater than array length
        if len(self.runner_points) > self.array_size:
            self.runner_points.pop(0)
            self.runner_times.pop(0)


    def publish_runner_history(self):
        # publishes runner history (IDK if necessary?)
        history_pose_array = PoseArray()
        history_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        history_pose_array.poses
    
        for point in self.runner_points:
            point_pose = Pose()
            point_pose.position.x = point.x
            point_pose.position.y = point.y
            history_pose_array.poses.append(point_pose)

        #print("Pred: Publishing particle cloud of size: " + str(len(self.runner_points)))

        self.path_pub.publish(history_pose_array)

    def publish_target_pose(self, target_point):
        new_pose = PoseStamped()
        new_pose.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        new_pose.pose = Pose()
        (x, y) = target_point
        new_pose.pose.position.x = x
        new_pose.pose.position.y = y

        self.target_pose_pub.publish(new_pose)


    def bumper_callback(self, data):
        # stop chasing if bumped into target
        if data.bumper == 2:
            self.bumped = True
            stop = Twist(
                    linear=Vector3(0, 0, 0),
                    angular=Vector3(0, 0, 0)
                    )
            r = rospy.Rate(2)
            t = 3
            while t > 0:
                self.twist_pub.publish(stop)
                r.sleep()
                t -= 1/2
            print("Bumped!")        


    def predict(self, pred_ts):
        pred_x = self.modelx.predict(np.array(pred_ts).reshape(-1,1))
        pred_y = self.modely.predict(np.array(pred_ts).reshape(-1,1))
        return pred_x, pred_y


    def run(self):
        # continuous loop that publishes vectors to move chaser in predicted path of runner
        r = rospy.Rate(2)
        
        
        while not rospy.is_shutdown() and not self.bumped:
            # wait until we have filled runner history
            if len(self.runner_points) >= 10:
                # get x and y paths separately
                xs = []
                ys = []

                for p in self.runner_points:
                    xs.append(p.x)
                    ys.append(p.y)

                # convert to arrays
                xs = np.array(xs).reshape(-1,1)
                ys = np.array(ys).reshape(-1,1)
                ts = np.array(self.runner_times).reshape(-1,1)
                ts = ts - ts[0] # make first timepoint 0
                print(f"coord : {(xs[-1][0], ys[-1][0], ts[-1][0])}")
                print("curr dist:", self.curr_distance)

                # predict next position of chaser proportionally to distance
                v = 0.05
                delta = 0.1 # basically how much more velocity to cover current distance
                pred_time = self.curr_distance / (v + delta) + ts[-1] # how far into future we want to predict, should be < set velocity

                # linear regression of x vs. t and y vs. t
                self.modelx = LinearRegression().fit(ts,xs)
                self.modely = LinearRegression().fit(ts,ys)
                pred_x, pred_y = self.predict(pred_time)
                self.publish_target_pose((pred_x[0][0], pred_y[0][0]))

                #print(f"x coef: {self.modelx.coef_}")
                #print(f"y coef: {self.modely.coef_}")
                print(f"pred coord: {(pred_x[0][0], pred_y[0][0], pred_time)}")

                # polynomial regression degree 3
                # poly = PolynomialFeatures(degree = 3)
                # pts = poly.fit_transform(ts)
                # polyx = LinearRegression().fit(pts, xs)
                # polyy = LinearRegression().fit(pts, ys)

                # print(f"x coef: {modelx.coef_}")
                # print(f"y coef: {modely.coef_}")

                # pred_x = polyx.predict(pred_time)
                # pred_y = polyy.predict(pred_time)

                # print(f"coordinate: {(pred_x, pred_y)}")

                # calculate proportional turn towards predicted point
                # linear velocity proportional to distance but has ceiling v
                # implement derivative or integral? I dont think so since we probably aren't moving fast
                dx = pred_x - self.curr_pose.position.x
                dy = pred_y - self.curr_pose.position.y
                
                pred_theta = math.atan2(dx, dy) # this might be issue if y and x are reversed
                pred_dist = math.sqrt(dx**2 + dy**2) 
                print("theta: ", pred_theta)
                print("pred dist: ", pred_dist)
                ka = 0.03
                kl = 0.05

                twist = Twist()
                twist.angular.z = ka * pred_theta
                twist.linear.x = min(kl * pred_dist, v)
                #print("publish twist:", twist)
                self.twist_pub.publish(twist)
                print("\n")

            r.sleep()
        
        rospy.spin()



if __name__ == "__main__":
    node = Prediction()
    node.run()
    