#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <termios.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "walk_forward_high_level_cmd_vel");

	ros::NodeHandle nh;

	ros::Rate loop_rate(500);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	geometry_msgs::Twist twist;

	long count = 0;

	while (ros::ok())
	{
		twist.linear.x = 0.5;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

		pub.publish(twist);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
