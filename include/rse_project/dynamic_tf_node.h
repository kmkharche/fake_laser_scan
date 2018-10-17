/* 
	@file - dynamic_tf_node.h
	@version - 0.0
	@date - July 16, 2018
	@brief - dynamic transform class definition
*/

#ifndef DYNAMIC_TF_NODE_H
#define DYNAMIC_TF_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class DynamicTf
{
	private:
		double count;
		tf::TransformBroadcaster mapToBase;
		tf::Transform transform;
		tf::Quaternion quart;
		std::string parent;
		std::string child;
		
	public:
		DynamicTf(std::string frame1="map",std::string frame2="laser");
		
		~DynamicTf();
		
		// Publishes a dynamic TF
		void broadcast();
};

#endif
