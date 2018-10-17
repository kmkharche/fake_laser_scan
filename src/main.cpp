/* 
	@file - main.cpp
	@version - 0.0
	@date - July 16, 2018
	@brief - main node calling the laser scan and dynamic tf publihsers
*/
 
#include "rse_project/scan_publisher_node.h"
#include "rse_project/dynamic_tf_node.h"

int main(int argc, char **argv)
{
	// Initialize the ROS node
	ros::init(argc,argv,"scan_publisher");
	ros::NodeHandle n;

	// Set up the frequency of the loop
	ros::Rate loop_rate(5.0);
	
	// Read and load parameters for the laser scan publisher
	int num_readings;
	n.param("num_readings",num_readings,1000);
	int laser_frequency;
	n.param("laser_frequency",laser_frequency,300000);
	double range_min;
	n.param("range_min",range_min,0.2);
	double range_max;
	n.param("range_max",range_max,10.0);
	std::string laser_frame;
	n.param<std::string>("laser_frame",laser_frame,"laser");
	
	// Create an object for the laser scan
	LaserScan *scan = new LaserScan(num_readings,laser_frequency,range_min,range_max,laser_frame);
	
	// Read and load parameters for the dynamic transform publisher
	std::string parent;
	n.param<std::string>("parent_frame",parent,"map");
	std::string child;
	n.param<std::string>("child_frame",child,"base_link");

	// Create an object for the dynamic transform
	DynamicTf *dynamic_tf = new DynamicTf(parent,child);
	
	// Set up a publisher for the fake circular laser scan
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan",100);
	
	// Set up a publisher for the fake random laser scan
	ros::Publisher scan_random_pub = n.advertise<sensor_msgs::LaserScan>("scan_random",100);
	
	while (ros::ok())
	{
		// Call the scan publisher functions
		scan->publisher(&scan_pub);
		scan->publisher_random(&scan_random_pub);
		
		// Call the dynamic tranform publisher
		dynamic_tf->broadcast();
		
		// Sleep to reach the correct loop frequency
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	// delete the dynamic pointers
	delete scan;
	scan = NULL;
	
	delete dynamic_tf;
	dynamic_tf = NULL;
	
	return 0;
}
