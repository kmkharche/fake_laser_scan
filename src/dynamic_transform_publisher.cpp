/* 
	@file - dynamic_transform_publisher.cpp
	@version - 0.0
	@date - July 16, 2018
	@brief - dynamic tf publihser
*/

#include "rse_project/dynamic_tf_node.h"

DynamicTf::DynamicTf(std::string frame1,std::string frame2):count(0),parent(frame1),child(frame2)
{
}
		
DynamicTf::~DynamicTf()
{
}

// Publishes a dynamic TF so that the robot moves along a square
void DynamicTf::broadcast()
{

	// Set up the translation and rotation values for motion along a square of size 20
	if (count<20)
	{
		transform.setOrigin(tf::Vector3(count,0.0,0.0));
		quart.setRPY(0.0,0.0,0.0);
	}
	else if (count<40)
	{
		transform.setOrigin(tf::Vector3(20.0,count-20.0,0.0));
		quart.setRPY(0.0,0.0,1.57);
	}
	else if (count<60)
	{
		transform.setOrigin(tf::Vector3(60.0-count,20.0,0.0));
		quart.setRPY(0.0,0.0,3.14);
	}
	else if (count<80)
	{
		transform.setOrigin(tf::Vector3(0.0,80.0-count,0.0));
		quart.setRPY(0.0,0.0,-1.57);
	}
	else
		count=0;
	
	// Broadcast the transform 	
	transform.setRotation(quart);
	mapToBase.sendTransform(tf::StampedTransform(transform,ros::Time::now(),parent,child));

	++count;
	
	// Reset count to zero once robot comes back to the starting point
	if (count>=80)
		count=0;
}
