#pragma once

#ifndef __PTAMINITIALIZER_H
#define __PTAMINITIALIZER_H

#include <stdlib.h>
#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ardrone_autonomy/Navdata.h>
#include "tum_ardrone/filter_state.h"
#include <hh_ardrone/NodeGen.h>
#include <time.h>


class PTAMInitializer
{
private:

	std::string clock_channel;
	std::string ToPTAM_channel;

	std::string Space_command = "p space";

	std_msgs::Empty empty_msg; //Empty Message publish to Land,Take Off and Emergency 

	ros::NodeHandle nh;  //Node Handler

	ros::Subscriber subClock;
	ros::Publisher pubToPTAM;

	int T1,T2;
	ros::Time t0;
	ros::Duration init1,init2;
	bool start_flag;
	bool first_space;
	bool second_space;

public:
	
	PTAMInitializer();  //Constructor
	void initPTAM();
	void checkTime();
};

#endif  /* __PTAMINITIALIZER_H */
