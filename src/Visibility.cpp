#include "include/Visibility.hpp"
#include "include/LambdaCell.hpp"
#include <std_msgs/Float32.h>
#include <numeric>
#include <cmath>

Visibility::Visibility(ros::NodeHandle &n):
mapper(n)
{
    int min_range, max_range;

	if (!n.getParam("/visibility/min_range", min_range)){
		ROS_ERROR("Fail to get minimum range param");
		exit(0);
	}
	
	// Get map_height parameter
	if (!n.getParam("/visibility/max_range", max_range)){
		ROS_ERROR("Fail to get maximum range param");
		exit(0);
	}

	if (!n.getParam("/sensor/aperture_angle", this->aperture_angle)){
		ROS_ERROR("Fail to get cell_size param");
		exit(0);
	}
	
	// Get map_height parameter
	if (!n.getParam("/visibility/probability", this->probability)){
		ROS_ERROR("Fail to get map_height param");
		exit(0);
	}
    // Compute the indices of the cells used for visibility
    int map_width = mapper.get_map_width();
    int map_height = mapper.get_map_height();
    float cell_size = mapper.get_cell_size();

    for (int i = 0 ; i < map_width ; i++) {
        for (int j = 0 ; j < map_height ; j++) {
           float x_cell = cell_size * (i - map_width / 2); 
           float y_cell = cell_size * (j - map_height / 2);
           float r_squared = x_cell * x_cell + y_cell * y_cell;

            if (r_squared > min_range * min_range && r_squared < max_range * max_range) {
                indices_in_range.push_back(std::make_pair(i,j)) ;
            }
        }
    }

    // Subscribe to laserscan input topic
	sub_laserScan = n.subscribe("laser_scan_in", 10, &Visibility::laserScan_callback, this);
    
    // Advertise to visibility output topic
	pub_visibility = n.advertise<std_msgs::Float32>("visibility_out", 2);
}


void Visibility::laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    // Compute lambda_map
    mapper.laserScan_callback(msg);
    const Map& map = mapper.get_map();

    // Get lambdas used to compute visibility 
    std::vector<float> lambda_values;
    for (std::vector<std::pair<int, int>>::iterator it = indices_in_range.begin() ; it != indices_in_range.end() ; it++){
        float lambda = (map[std::get<0>(*it)][std::get<1>(*it)]).lambda();
        if (lambda != LambdaCell::get_lambda_unmeasured()){
            lambda_values.push_back(lambda); 
        }
    }
    
    // Compute visibility, returns -1 if no lambda values available 
    float avg_lambda = lambda_values.size() == 0 ? -1. : (float) std::accumulate(lambda_values.begin(), lambda_values.end(), 0) / lambda_values.size() ;
    float visibility = avg_lambda == -1. ? -1. : sqrt(-2. * log (probability) /( aperture_angle * avg_lambda)) ;

    // Publish visibility value
    std_msgs::Float32 visibility_msg;
    visibility_msg.data = visibility;
    pub_visibility.publish(visibility_msg);
}


