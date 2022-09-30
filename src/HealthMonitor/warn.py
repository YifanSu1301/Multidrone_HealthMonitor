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


class WarningSystem():
    
    def __init__(self):
        print("Starting Warning System")
        # 1. Initialization

        # 1a. Initialize Condition Status
        self.receive_debug_info = Bool()
        self.receive_debug_info = False



        # 2. Subscribe debug_info
        self.odom_status_sub = rospy.Subscriber('debug_info', Health, self.warnError)
        

    # 3. Check if `debug_info` received
    def check_receive(self):
        if(not self.receive_debug_info):
            print("No topic called `debug_info`")

    # 4. Callback functions
    def warnError(self,data):
        self.receive_debug_info = True
        # 3a. check the number of satellites
        sat1_num = data.sat1_num
        sat2_num = data.sat2_num
        if sat1_num < 10:
            print("WARNING: GNSS_1 has less then 10 satellites: {}".format(sat1_num))
        if sat2_num < 10:
            print("WARNING: GNSS_2 has less then 10 satellites: {}".format(sat2_num))

        # 3b. check the covariance value 
        sat1_covariance = data.sat1_covariance
        sat2_covariance = data.sat2_covariance
        if sat1_covariance[0] > 0.0004 or sat1_covariance[4] > 0.0004 or sat1_covariance[8] > 0.0004:
            print("WARNING: GNSS 1 has covariance larger than 0.0004, {}.".format(sat1_covariance))
        if sat2_covariance[0] > 0.0004 or sat2_covariance[4] > 0.0004 or sat2_covariance[8] > 0.0004:
            print("WARNING: GNSS 2 has covariance larger than 0.0004, {}.".format(sat2_covariance))

        # 3c. check the RTK 
        if data.RTK_status != "Received":
            print("WARNING: RTK is not working properly.")

    

  


if __name__ == '__main__':
    rospy.init_node('warn_node')
    rate = rospy.Rate(1)
    node = WarningSystem()
    try:
        while not rospy.is_shutdown():
            node.check_receive()
            rate.sleep()
    except rospy.ROSInterruptException:
        raise
    print('Success!')