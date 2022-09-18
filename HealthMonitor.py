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
        self.gnss1_status_sub = rospy.Subscriber('/gnss1/fix_info', Odometry, self.gnss1_callback)
        self.gnss2_status_sub = rospy.Subscriber('/gnss2/fix_info', Odometry, self.gnss2_callback)
        self.rtk_status_sub = rospy.Subscriber('/rtk/status_v1', Odometry, self.rtk_callback)

    def odom_callback(self,data):
        self.odom_pub_status.data = True
    def gnss1_callback(self,data):
        self.gnss1_pub_status.data = True
    def gnss2_callback(self,data):
        self.gnss2_pub_status.data = True
    def rtk_callback(self,data):
        self.rtk_pub_status.data = True
    

    def getStatus(self):
        status = (self.odom_pub_status.data and self.gnss1_pub_status.data and 
                    self.gnss2_pub_status.data and self.rtk_pub_status.data)
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