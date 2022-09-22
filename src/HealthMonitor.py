#!/usr/bin/env python3

from logging import raiseExceptions
import rospy
import time
import os
from mf_gps_backpack.msg import Health
from std_msgs.msg import Bool, UInt32, String
from sensor_msgs.msg import NavSatFix 
from nav_msgs.msg import Odometry
from microstrain_inertial_msgs.msg import GNSSAidingStatus, GNSSDualAntennaStatus, FilterStatus, RTKStatusV1, GNSSFixInfo

class HealthMonitor():
    
    def __init__(self):
        print("Starting Health Monitor")
        # 1. Initialization

        # 1a. Initialize Condition Status
        self.gnss1_pub_status = Bool()
        self.gnss2_pub_status = Bool()
        self.odom_pub_status = Bool()
        self.rtk_pub_status = Bool()
        self.gnss1_aiding = Bool()
        self.gnss2_aiding = Bool()
        self.filter = Bool()
        self.gnss1_fix = Bool()
        self.gnss2_fix = Bool()

        self.gnss1_pub_status.data = False
        self.gnss2_pub_status.data = False
        self.odom_pub_status.data = False
        self.rtk_pub_status.data = False
        self.gnss1_aiding.data = False
        self.gnss2_aiding.data = False
        self.filter.data = False
        self.gnss1_fix.data = False
        self.gnss2_fix.data = False

        # 1b. Initialize Debug Data Structure
        self.debug_msg = Health() 

        # 1c. Initialize Overall Publisher
        robotName = os.uname()[1] # get the system host name
        self.OverallSuccess_pub = rospy.Publisher( '/overall_success', Bool, queue_size = 1)

        # 1d. Initialize Dbug Message Publisher
        self.debug_info_pub = rospy.Publisher( '/debug_info', Health, queue_size = 1)

        # 2. Subscribe topics
        self.odom_status_sub = rospy.Subscriber('/odom_out', Odometry, self.odom_callback)
        self.gnss1_status_sub = rospy.Subscriber('/gnss1/fix_info', GNSSFixInfo, self.gnss1_fixinfo_callback)
        self.gnss2_status_sub = rospy.Subscriber('/gnss2/fix_info', GNSSFixInfo, self.gnss2_fixinfo_callback)
        self.rtk_status_sub = rospy.Subscriber('/rtk/status_v1', RTKStatusV1, self.rtk_callback)
        self.gnss1_aiding_sub = rospy.Subscriber('/gnss1/aiding_status', GNSSAidingStatus, self.gnss1_aiding_callback)
        self.gnss2_aiding_sub = rospy.Subscriber('/gnss2/aiding_status', GNSSAidingStatus, self.gnss2_aiding_callback)
        self.filter_sub = rospy.Subscriber('/nav/status', FilterStatus, self.filter_callback)
        self.gnss1_fix_sub = rospy.Subscriber('/gnss1/fix', NavSatFix, self.gnss1_fix_callback)
        self.gnss2_fix_sub = rospy.Subscriber('/gnss2/fix', NavSatFix, self.gnss2_fix_callback)

    # Callback functions
    def odom_callback(self,data):
        self.odom_pub_status.data = True

    def gnss1_fixinfo_callback(self,data):
        self.debug_msg.sat1_num = data.num_sv
        if(self.debug_msg.sat1_num > 12) :
            self.gnss1_pub_status.data = True

    def gnss2_fixinfo_callback(self,data):
        self.debug_msg.sat2_num = data.num_sv
        if(self.debug_msg.sat2_num > 12) :
            self.gnss2_pub_status.data = True

    def rtk_callback(self,data):
        self.debug_msg.RTK_status = "Received"
        self.rtk_pub_status.data = True

    def gnss2_aiding_callback(self, data):
        self.gnss2_aiding.data = True

    def gnss1_aiding_callback (self, data):
        self.gnss1_aiding.data = True

    def filter_callback (self, data):
        self.filter.data = True

    def gnss1_fix_callback (self, data):
        self.debug_msg.sat1_covariance = data.position_covariance
        c_type = data.position_covariance_type

        if c_type == 1:
            self.debug_msg.sat1_cov_type = "APPROXIMATED"
        elif c_type == 2:
            self.debug_msg.sat1_cov_type = "DIAGONAL_KNOWN"
        elif c_type == 3:
            self.debug_msg.sat1_cov_type = "KNOWN"
        else:
            self.debug_msg.sat1_cov_type = "UNKNOWN"

        if(c_type == "DIAGONAL_KNOWN"):
            self.gnss1_fix.data = True

    def gnss2_fix_callback (self, data):
        self.debug_msg.sat2_covariance = data.position_covariance
        c_type = data.position_covariance_type

        if c_type == 1:
            self.debug_msg.sat2_cov_type = "APPROXIMATED"
        elif c_type == 2:
            self.debug_msg.sat2_cov_type = "DIAGONAL_KNOWN"
        elif c_type == 3:
            self.debug_msg.sat2_cov_type = "KNOWN"
        else:
            self.debug_msg.sat2_cov_type = "UNKNOWN"
        if(c_type == "DIAGONAL_KNOWN"):
            self.gnss2_fix.data = True

    

    def getStatus(self):
        status = (self.odom_pub_status.data and self.gnss1_pub_status.data and 
                    self.gnss2_pub_status.data and self.rtk_pub_status.data and 
                    self.gnss2_aiding.data and self.gnss1_aiding.data and 
                    self.filter.data and self.gnss1_fix.data and self.gnss2_fix.data)
        return status


if __name__ == '__main__':
    rospy.init_node('startup_status')
    rate = rospy.Rate(1)
    node = HealthMonitor()
    try:
        while not rospy.is_shutdown():
            status = node.getStatus()
            node.OverallSuccess_pub.publish(status)
            node.debug_info_pub.publish(node.debug_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        raise
    print('Success!')