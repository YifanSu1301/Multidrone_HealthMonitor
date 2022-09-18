#!/usr/bin/env python3

from logging import raiseExceptions
import rospy
import time
import os
from std_msgs.msg import Bool, UInt32, String
from sensor_msgs.msg import NavSatFix 
from nav_msgs.msg import Odometry
from microstrain_inertial_msgs.msg import GNSSAidingStatus, GNSSDualAntennaStatus, FilterStatus, RTKStatus, GNSSFixInfo

class HealthMonitor():
    
    def __init__(self):
        print("Starting Health Monitor")
        # 1. Initialization

        # 1a. Initialize Status
        self.gnss1_pub_status = Bool()
        self.gnss2_pub_status = Bool()
        self.odom_pub_status = Bool()
        self.rtk_pub_status = Bool()

        self.gnss1_pub_status.data = False
        self.gnss2_pub_status.data = False
        self.odom_pub_status.data = False
        self.rtk_pub_status.data = False

        # 1b. Initialize Overall Publisher
        robotName = os.uname()[1] # get the system host name
        self.OverallSuccess_pub = rospy.Publisher( '/overall_success', Bool, queue_size = 1)

        # 2. Subscribe topics
        self.odom_status_sub = rospy.Subscriber('/odom_out', Odometry, self.odom_callback)

    def odom_callback(self,data):
        self.odom_pub_status.data = True

    def getStatus(self):
        status = self.odom_pub_status.data 
        return status


if __name__ == '__main__':
    rospy.init_node('startup_status')
    rate = rospy.Rate(1)
    node = HealthMonitor()
    try:
        while not rospy.is_shutdown():
            status = node.getStatus()
            node.OverallSuccess_pub.publish(status)
            rate.sleep()
    except rospy.ROSInterruptException:
        raise
    print('Success!')