/* 
	@file - scan_publisher_node.h
	@version - 0.0
	@date - July 16, 2018
	@brief - laser scan class definition
*/

#ifndef SCAN_PUBLISHER_NODE_H
#define SCAN_PUBLISHER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>

class LaserScan
{
	private:
		int state;		// Used by the publisher function below
		int count;
		int readings;	// Number of readings in one scan
		int frequency;	// Frequence of points scanned by lidar
		int min_range;
		int max_range;
		std::string frame;
		
	public:
	
		LaserScan(int num_readings=1000, int laser_frequency=100000, double range_min=0.2, double range_max=30.0, std::string frame="laser");
		
		~LaserScan();
		
		// Publishes a fake circular laser scan of continuously increasing or continuously decreasing radius
		// The state variable above dettermines if the circle is increasing or decreasing
		void publisher(ros::Publisher *pub);
		
		// Publishes a fake random laser scan
		void publisher_random(ros::Publisher *pub_random);
};

#endif

