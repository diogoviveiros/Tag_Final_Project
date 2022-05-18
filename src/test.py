#!/usr/bin/env python3

import rospy


from sensor_msgs.msg import LaserScan

class Laser(object):
    
    
    def __init__(self):
        # Start rospy node.
        print("Initializing")
        rospy.init_node("follow_person")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)


    def process_scan(self, data):

        print("In process_scan")

        print(data.ranges[0])
        print(data.ranges[1])
        print(data.ranges[358])
        print(data.ranges[359])
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    laser = Laser()
    laser.run()

