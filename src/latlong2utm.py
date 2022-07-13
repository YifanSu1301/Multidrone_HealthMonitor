#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import utm
import sys

# Callback on a PointStamped message representing the location of an object
# defined with lat long coordinates
# Outputs a converted version in UTM coordinates in the form of a UTMStamped message type
def local_position_callback(msg):
    lat = msg.pose.pose.position.x
    lon = msg.pose.pose.position.y
    easting, northing, zone_num, zone_let = utm.from_latlon(lat, lon)

    msg.pose.pose.position.x = easting
    msg.pose.pose.position.y = northing

    utmPub.publish(msg)


if __name__ == "__main__":
    # Subscribe to a Geometry_msgs/PointStamped representing
    # lat, long, and altitude and publish the UTM equivalent
    rospy.init_node("latlong2utm", anonymous=True)

    rospy.myargv(argv=sys.argv)

    # Might want to revisit instead of command line arguments
    # sub_topic = rospy.get_param("~latlon_topic", "filtered_odom")
    # pub_topic = rospy.get_param("~utm_topic", "/m3/latlong2utm/odom_utm")

    sub_topic = sys.argv[1]
    pub_topic = sys.argv[2]

    rospy.Subscriber(sub_topic, Odometry, local_position_callback)
    utmPub = rospy.Publisher(pub_topic, Odometry, queue_size=10)

    rospy.spin()

