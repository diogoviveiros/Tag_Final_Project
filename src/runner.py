#!/usr/bin/env python3

import rospy
import math
import random

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class Runner(object):

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('runner')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
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



if __name__ == '__main__':
    # instantiate the ROS node and run it
    runner = Runner()
    runner.run()