#pragma once

#include <ros/ros.h>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <lidar_visibility/LambdaGrid.h>
#include "NormalEstimator.hpp"
#include "LambdaCell.hpp"

typedef std::vector<std::vector<LambdaCell>> Map ;

class LambdaMapper
{
  public:
	LambdaMapper(ros::NodeHandle &n);
	~LambdaMapper();

	void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void publish_occupancyGrid();
	void publish_lambdaGrid();

	void publish(const ros::TimerEvent&);

	void evolve_map(const ros::Time t);

    const Map& get_map() const {return map;}
    int get_map_width() const {return map_width;}
    int get_map_height() const {return map_height;}
    float get_cell_size() const {return cell_size;} 
		
  private:
	geometry_msgs::TransformStamped base_linkTlaser;

	float cell_size;
	int map_width;	
	int map_height;
	int robotOffsetMap;
	float angular_filter;
	float radius_filter;
	std::array<float,2> error_region;
	bool rotation_map;

	NormalEstimator normal_estimator;
	Map map;

	tf2::Quaternion orientation; //orientation of the map

	//ROS SPECIFIC
	ros::Subscriber sub_odom;
	ros::Publisher pub_occupancyGrid;
	ros::Publisher pub_lambdaGrid;

	tf2_ros::Buffer tfBuff;
	tf2_ros::TransformListener listener;

	std::string sensor_frame;
	std::string odom_frame;
	std::string robot_frame;

	ros::Timer timer;
};

template<typename T>
std::int8_t sign(const T var){
	return (var > 0) - (var < 0);
}
