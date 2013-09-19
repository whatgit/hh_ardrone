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

	// Declare variables that can be modified by launch file or command line.
	double Threshold;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can
	// be run simultaneously while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("Threshold", Threshold, 3.0);

	cout << "set Threshold to " << Threshold << "m"<< endl;
		
	//-- Initialize Variables --//
	lastTagCount = 0;
	nodeCount = 0;
	distanceThreshold = Threshold;

	Tag = new tag_info;
	Tag->tag_found = false;

	current = &node_array[0];
	
}

/*If it is a new tag return true, else return false*/
bool NodeGenerator::addTagNode(int drone_x, int drone_y, int tag_xc, int tag_yc, float angle)
{
	if(nodeCount == 0)
	{
		nodeCount = nodeCount+1;
		current = &node_array[nodeCount];
		current->tag_x = drone_x;
		current->tag_y = drone_y;
		current->tag_xc = tag_xc;
		current->tag_yc = tag_yc;
		lastTagAngle = angle;
		return true;
	}
	else
	{
		//if((abs(drone_x-(current->tag_x)) > distanceThreshold) || (abs(drone_y-(current->tag_y)) > distanceThreshold))
		if(abs(lastTagAngle-angle) > 120.00)
		{
			nodeCount = nodeCount+1;
			current = &node_array[nodeCount];
			current->tag_x = drone_x;
			current->tag_y = drone_y;
			current->tag_xc = tag_xc;
			current->tag_yc = tag_yc;
			lastTagAngle = angle;
			return true;
		}
		else	//Still old one
		{
			return false;
		}
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
		if(addTagNode(state.x, state.y, Tag->xc, Tag->yc, Tag->tag_angle))
		{
			camPix2CorWorldFrame(state.x, state.y, state.z);
			ROS_INFO("Tag no. %d found at drone x : %.10f , y : %.10f and xc : %d, yc : %d, tagWorldX : %.10f, tagWorldY : %.10f",nodeCount , state.x, state.y, Tag->xc, Tag->yc, tagWorldX, tagWorldY);
			node.tag_no = nodeCount;
			node.x = state.x;
			node.y = state.y;
			node.z = state.z;
			node.tag_x = tagWorldX;
			node.tag_y = tagWorldY;
			pubNodegen.publish(node);
		}	
		Tag->tag_found = false;
	}
	
}

void NodeGenerator::navCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	NavdataReceived = *navdataPtr;
	//if(NavdataReceived.tags_count == 1 && lastTagCount == 0 && NavdataReceived.state == 4)
	if(NavdataReceived.tags_count == 1 && lastTagCount == 0)
	{
		//ROS_INFO("Found Tag !!");
		Tag->xc = NavdataReceived.tags_xc[0];
		Tag->yc = NavdataReceived.tags_yc[0];
		Tag->tag_found = true;
		Tag->tag_angle = NavdataReceived.tags_orientation[0];
		lastTagCount = NavdataReceived.tags_count;
	}
	
	if(NavdataReceived.tags_count == 0 && lastTagCount == 1 && Tag->tag_found == false)
	{
		//ROS_INFO("Reset Tag Count");
		lastTagCount = 0;
	}
	
	//lastTagCount = NavdataReceived.tags_count;
}

void NodeGenerator::camPix2CorWorldFrame(float x, float y, float z){
	
	float Mcx = 0.871576/2.0;
	float Mcy = 0.493151/2.0;

	tagWorldX = x + (((((float) Tag->xc / 1000)*640)-320.0)*((2.0*z)/734.3));
	tagWorldY = y + ((180.0-(((float) Tag->yc / 1000)*360))*((2.0*z)/734.3));

	//tagWorldX = x + (((float) Tag->xc - 500)/1000)*(Mcx*z*2);
	//tagWorldY = y + (((float) Tag->yc - 500)/1000)*(Mcy*z*2);
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
