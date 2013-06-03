#pragma once

#ifndef __NODEGENERATOR_H
#define __NODEGENERATOR_H

#include <stdlib.h>
#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include "tum_ardrone/filter_state.h"
#include <hh_ardrone/NodeGen.h>

#define MAX_NODE 10

class NodeGenerator
{
private:

	std::string dronepose_channel;
	std::string navdata_channel;
	std::string nodeout_channel;

	std_msgs::Empty empty_msg; //Empty Message publish to Land,Take Off and Emergency 

	ros::NodeHandle nh;  //Node Handler

	ros::Subscriber subPose;
	ros::Subscriber subNavdata;
	ros::Publisher pubNodegen;

	ardrone_autonomy::Navdata NavdataReceived;

	struct tag_node{
	  int tag_x;
	  int tag_y;
	  int tag_xc;
	  int tag_yc;
	};

	struct tag_info{
	  int xc;
	  int yc;
	  bool tag_found;
	};
	
	tag_info *Tag;
	int currentNode;
	int lastTagCount;
	int distanceThreshold;

	void addTagNode(int drone_x, int drone_y, int tag_xc, int tag_yc);

public:

	int nodeCount;
	tag_node *current;
	tag_node node_array[MAX_NODE];

	NodeGenerator();  //Constructor
	void posCb(const tum_ardrone::filter_stateConstPtr);
	void navCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);	
};

#endif  /* __DRONECONTROL_H */
