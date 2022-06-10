#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

void imuDataCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
	ROS_INFO("Linear Acceleration: [%f, %f, %f]", imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener");

	std::string deviceName;
	ros::NodeHandle params("~");
	params.param<std::string>("device", deviceName, "gx5");
	ROS_INFO("Got device param: %s", deviceName.c_str());

	params.deleteParam("device");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(("/imu/data"),3,imuDataCallback);
	ros::spin();
	return 0;
}
