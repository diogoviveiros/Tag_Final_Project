#!/usr/bin/env python3

import rospy
import math
import random

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
    
    def run(self):

        # setup the Twist message that moves the robot forward
        move_forward = Twist(
            linear=Vector3(0.5, 0, 0),
            angular=Vector3(0, 0, 0)
        )

        rotate = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0, random.uniform(-3.14, 3.14) )
        )

        self.robot_movement_pub.publish(rotate)

        rospy.sleep(2)

        self.robot_movement_pub.publish(move_forward)

        rospy.sleep(random.randrange(4,9))

    def process_scan(self, data):

        min_dist = 5
        
        #Checks the front of the robot for an obstacle (particularly walls)
        if(data.ranges[0] <= min_dist and data.ranges[0] != 0.0):
            angle = random.uniform(-4.71, 4.71)
            while(math.abs(angle) < 4.71): 
                angle = random.uniform(-4.71, 4.71)

            self.twist.angular.z = angle

         
        #Rotate in a random direction predetermined in the if condition right above
    
            
        self.twist_pub.publish(self.twist)
        rospy.delay(2)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    runner = Runner()
    runner.run()