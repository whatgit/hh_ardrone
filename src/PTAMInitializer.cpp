#include "PTAMInitializer.h"

using namespace std;



PTAMInitializer::PTAMInitializer()
{
	ToPTAM_channel = nh.resolveName("tum_ardrone/com");
	clock_channel = nh.resolveName("clock");
	
	pubToPTAM = nh.advertise<std_msgs::String>(ToPTAM_channel,50);
	
	// Declare variables that can be modified by launch file or command line.
	int T1,T2;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can
	// be run simultaneously while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("T1", T1, 0);
	private_node_handle_.param("T2", T2, 0);

	cout << "set initialize time T1 to " << T1 << " and T2 to " << T2 << " second"<< endl;
	start_flag = false;
	first_space = false;
	second_space = false;
	
	init1 = ros::Duration(T1,0);
	init2 = ros::Duration(T2,0);
}

void PTAMInitializer::checkTime(){
	ros::Time time;
	time = ros::Time::now();
	if(time.sec != 0)	//Should be 0 until it get first clock
	{
	  if(!start_flag)
	  {
	     start_flag = true;
  	     t0 = time;
	     cout << "Start time is : " << t0.toSec() << endl;
	  }
	  if((time >= t0+init1) && !first_space)
	  {
	     initPTAM();
	     first_space = true;
	  }
	  if((time >= t0+init2) && !second_space)
	  {
	     initPTAM();
	     second_space = true;
	  }
	  if(first_space && second_space)
	  {
	     ros::shutdown();
          }
	  //cout << "check time" << t0.toSec() << " plus " << T2 << " = " << t0.toSec()+T2 << endl;
	}
}


void PTAMInitializer::initPTAM(){
	std_msgs::String s;
	s.data = Space_command.c_str();
	//pthread_mutex_lock(&tum_ardrone_CS);
	pubToPTAM.publish(s);
	//pthread_mutex_unlock(&tum_ardrone_CS);
	cout << "Init PTAM"<< endl;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ptam_init");
  PTAMInitializer ptam_init;
  ros::Rate loop_rate(50);

  ROS_INFO("Started PTAM Initializer.\r\n");

  while (ros::ok()) {
	ptam_init.checkTime();
	ros::spinOnce();	
	loop_rate.sleep();
  }

  return 0;
}
