#!/usr/bin/env python3

import rospy
import random
from datetime import datetime

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class Runner(object):

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('runner')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #Setup publisher for laser scanning
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.process_scan, queue_size =1)
        self.last_turn_time = datetime.now()

        self.last_update = datetime.now()

        self.twist = Twist(
                    linear=Vector3(0.05, 0, 0),
                    angular=Vector3(0, 0, 0)
                )

        self.last_twist = None


        rospy.sleep(2)
    
    def run(self):
            print("IS RUNNING")
            # setup the Twist message that moves the robot forward
            
        #rospy.spin()


    def process_scan(self, data):
            min_dist = 0.4
            
            #Checks the front of the robot for an obstacle (particularly walls)
            if(data.ranges[0] <= min_dist and data.ranges[0] != 0.0):
                print("Wall")
                angle = -2
                self.twist.linear.x = 0
                self.twist.angular.z = angle
            else:
                print("No wall")
                self.twist.linear.x = 0.05

                if ((datetime.now() - self.last_turn_time).total_seconds() > 12):
                    self.twist.angular.z = random.uniform(-1, 1)
                    self.twist.linear.x = 0
                    self.last_turn_time = datetime.now()

                if ((datetime.now() - self.last_turn_time).total_seconds() > 2):
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0.05
        
         
            #Rotate in a random direction predetermined in the if condition right above
            if self.twist != self.last_twist:
                print("Publishing new twist")
                print(self.twist)
                self.robot_movement_pub.publish(self.twist)
                self.last_twist = self.last_twist

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # instantiate the ROS node and run it
    runner = Runner()
    runner.run()
