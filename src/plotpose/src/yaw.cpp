#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include "math.h"
double roll;
double pitch;
double yaw;
geometry_msgs::Vector3Stamped angle;
void calib(const sensor_msgs::Imu::ConstPtr& msg)
{

	//rotation from quaternion to euler angle
	tf::Quaternion Q(
	msg->orientation.x,
	msg->orientation.y,
	msg->orientation.z,
	msg->orientation.w);

	tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}



int main(int argc , char **argv)
{

	ros::init(argc , argv , "yaw");	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::Imu> ("/dji_sdk/imu",2,calib);
	ros::Publisher pub = n.advertise<geometry_msgs::Vector3Stamped>("/m100/yaw", 1);
	ros::Rate loop_rate(60);
	ROS_INFO("Hi\n");
	while (ros::ok())
	{
	angle.vector.x=roll*180/3.1415926;
	angle.vector.y=pitch*180/3.1415926;
	angle.vector.z=yaw*180/3.1415926;
	angle.header.stamp = ros::Time::now();
    	pub.publish(angle);
	ros::spinOnce();
	loop_rate.sleep();
	}
	
        return 0;
}

