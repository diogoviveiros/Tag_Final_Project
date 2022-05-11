#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node drives the robot in a square clockwise"""

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def run(self):
        # setup the Twist message that moves the robot forward
        move_forward = Twist(
            linear=Vector3(0.5, 0, 0),
            angular=Vector3(0, 0, 0)
        )

        # setup the Twist message that turns the robot right 90 degrees
        # 1.57 radian is 90 degrees, since there is friction on the ground, I increase it to 1.63.
        # The negative sign of the radian turns the robot to the right direction.
        turn_right = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, -1.63)
        )

        # setup the Twist message that stops the robot
        stop = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
        )

        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

        # publish the messages 
        # the robot would first move forward, then turn right 90 degrees. 
        # These two actions would be completed by the robot for 4 times.
        self.robot_movement_pub.publish(move_forward)

        rospy.sleep(1)

        self.robot_movement_pub.publish(turn_right)

        rospy.sleep(1)

        self.robot_movement_pub.publish(move_forward)

        rospy.sleep(1)

        self.robot_movement_pub.publish(turn_right)

        rospy.sleep(1)

        self.robot_movement_pub.publish(move_forward)

        rospy.sleep(1)

        self.robot_movement_pub.publish(turn_right)

        rospy.sleep(1)

        self.robot_movement_pub.publish(move_forward)
        
        rospy.sleep(1)
        
        self.robot_movement_pub.publish(turn_right)

        rospy.sleep(1)

        # robot comes to a full stop at the end
        self.robot_movement_pub.publish(stop)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()
