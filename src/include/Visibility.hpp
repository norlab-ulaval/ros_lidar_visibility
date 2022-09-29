#pragma once
#include "ros/ros.h"
#include "LambdaMapper.hpp"

class Visibility
{
public:
    Visibility (ros::NodeHandle &n);
    void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
    LambdaMapper mapper;
    
    float probability;
    float aperture_angle;

    std::vector<std::pair<int, int>> indices_in_range;

    ros::Subscriber sub_laserScan;
    ros::Publisher pub_visibility;

};
