/* 
	@file - scan_publisher_node.cpp
	@version - 0.0
	@date - July 16, 2018
	@brief - laser scan publihser
*/

#include "rse_project/scan_publisher_node.h"

LaserScan::LaserScan(int num_readings,int laser_frequency,double range_min,double range_max,std::string frame_id):state(0),count(0),readings(num_readings),frequency(laser_frequency),min_range(range_min),max_range(range_max),frame(frame_id)
{
	// Initialize random seed for the random laser scan
	srand((unsigned)time(0));
}

LaserScan::~LaserScan()
{
}

// Publishes a fake circular laser scan
void LaserScan::publisher(ros::Publisher *pub)
{

	// Current time for the header
	ros::Time timestamp = ros::Time::now();
	
	// Populate the fields of the scan message
	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.stamp = timestamp;
	scan_msg.header.frame_id = frame;
	scan_msg.angle_min = -3.14;
	scan_msg.angle_max = 3.14;
	scan_msg.angle_increment = (2*3.14)/readings;
	scan_msg.time_increment = (readings/frequency);
	scan_msg.range_min = min_range;
	scan_msg.range_max = max_range;
	
	// Resize ranges and intensties to match the number of readings
	scan_msg.ranges.resize(readings);
	scan_msg.intensities.resize(readings);

	for (int i=0;i<readings;++i)
	{
		scan_msg.ranges[i] = count;
		scan_msg.intensities[i] = 1;
	}
	
	// Increase the circle radius
	if (!state)
	{
		if (count<20)
			++count;
		else
		{
			--count;
			state=1;
		}
	}
	// Or else decrease the circle radius
	else
	{
		if (count>0)
			--count;
		else
		{
			++count;
			state=0;
		}
	}
	
	// Publish the message
	pub->publish(scan_msg);
}

// Publishes a fake random laser scan
void LaserScan::publisher_random(ros::Publisher *pub_random)
{
	// Current time for the header
	ros::Time timestamp = ros::Time::now();

	// Populate the fields of the scan message
	sensor_msgs::LaserScan scan_random_msg;
	scan_random_msg.header.stamp = timestamp;
	scan_random_msg.header.frame_id = frame;
	scan_random_msg.angle_min = -3.14;
	scan_random_msg.angle_max = 3.14;
	scan_random_msg.angle_increment = (2*3.14)/readings;
	scan_random_msg.time_increment = (readings/frequency);
	scan_random_msg.range_min = min_range;
	scan_random_msg.range_max = max_range;

	// Resize ranges and intensties to match the number of readings
	scan_random_msg.ranges.resize(readings);
	scan_random_msg.intensities.resize(readings);
	
	// Randomize the scan into 24 intervals
	int x=readings/18;
	
	for (int i=0;i<readings-x;i+=x)
	{
		// Generate a random value between the minimum and maximum range of the laser and random intensity between 0 and 1
		double range = min_range + ((double)rand()/RAND_MAX)*(max_range - min_range);
		double intensity = (double)rand()/(RAND_MAX);
		
		// Assign this value to one interval
		for (int j=i;j<i+x;++j)
		{
			scan_random_msg.ranges[j]=range;
			scan_random_msg.intensities[j]=intensity;
		}
	}
	
	// Publish the message
	pub_random->publish(scan_random_msg);
}
