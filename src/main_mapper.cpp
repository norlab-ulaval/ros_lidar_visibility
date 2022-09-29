#include "include/Visibility.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lambda_mapper");
	ros::NodeHandle n;

	Visibility vis(n);

	ros::spin();

	return 0;
}
