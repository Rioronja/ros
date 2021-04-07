#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include "math.h"
double x;
double y;
double z;
void getgps(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	ROS_INFO("x: %f\n", msg->point.x);
	ROS_INFO("y: %f\n", msg->point.y);
	ROS_INFO("z: %f\n", msg->point.z);
}



int main(int argc , char **argv)
{

	ros::init(argc , argv , "yaw");	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<geometry_msgs::PointStamped> ("/dji_sdk/local_position", 3, getgps);
	ros::Rate loop_rate(5); // 5Hz
	ROS_INFO("Hi\n");
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
        return 0;
}

