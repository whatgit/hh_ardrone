#include "NodeGenerator.h"

using namespace std;



NodeGenerator::NodeGenerator()
{
	dronepose_channel = nh.resolveName("ardrone/predictedPose");
	navdata_channel = nh.resolveName("ardrone/navdata");
	nodeout_channel = nh.resolveName("hh_ardrone/node_gen");
	
	subPose = nh.subscribe(dronepose_channel,10, &NodeGenerator::posCb, this);
	subNavdata = nh.subscribe(navdata_channel,10, &NodeGenerator::navCb, this);
	pubNodegen   = nh.advertise<hh_ardrone::NodeGen>(nodeout_channel,1);
	
	//-- Initialize Variables --//
	lastTagCount = 0;
	nodeCount = 0;
	distanceThreshold = 0.5;

	Tag = new tag_info;
	Tag->tag_found = false;

	current = &node_array[0];
	
}

void NodeGenerator::addTagNode(int drone_x, int drone_y, int tag_xc, int tag_yc)
{
	if((abs(drone_x-(current->tag_x)) > distanceThreshold) || (abs(drone_y-(current->tag_y)) > distanceThreshold))
	{
		nodeCount = nodeCount+1;
		current = &node_array[nodeCount];
		current->tag_x = drone_x;
		current->tag_y = drone_y;
		current->tag_xc = tag_xc;
		current->tag_yc = tag_yc;
	}
	else	//Still old one
	{
		current->tag_x = (current->tag_x+drone_x)/2;
		current->tag_y = (current->tag_y+drone_y)/2;
		current->tag_xc = (current->tag_xc+tag_xc)/2;
		current->tag_yc = (current->tag_yc+tag_yc)/2;
	}
}
	
void NodeGenerator::posCb(const tum_ardrone::filter_stateConstPtr pose)
{
	tum_ardrone::filter_state state = *pose;
	hh_ardrone::NodeGen node;

	//ROS_INFO("Pose Estimation Callback !!!\r\n");
	if(Tag->tag_found)
	{
		//ROS_INFO("foundTag is true !");
		addTagNode(state.x, state.y, Tag->xc, Tag->yc);
		ROS_INFO("Tag no. %d found at drone x : %.10f , y : %.10f and xc : %d, yc : %d",nodeCount , state.x, state.y, Tag->xc, Tag->yc);
		node.tag_no = nodeCount;
		node.x = state.x;
		node.y = state.y;
		node.z = state.z;
		pubNodegen.publish(node);	
		Tag->tag_found = false;
	}
	
}

void NodeGenerator::navCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	NavdataReceived = *navdataPtr;
	if(NavdataReceived.tags_count == 1 && lastTagCount == 0 && NavdataReceived.state == 4)
	{
		ROS_INFO("Found Tag !!");
		Tag->xc = NavdataReceived.tags_xc[0];
		Tag->yc = NavdataReceived.tags_yc[0];
		Tag->tag_found = true;
		lastTagCount = NavdataReceived.tags_count;
	}
	
	if(NavdataReceived.tags_count == 0 && lastTagCount == 1 && Tag->tag_found == false)
	{
		ROS_INFO("Reset Tag Count");
		lastTagCount = 0;
	}
	
	//lastTagCount = NavdataReceived.tags_count;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "node_generator");
  NodeGenerator node_gen;
  ros::Rate loop_rate(50);

  ROS_INFO("Started Node Generator.\r\n");

  while (ros::ok()) {
	ros::spinOnce();	
	loop_rate.sleep();
  }

  return 0;
}
