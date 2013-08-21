#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include "tum_ardrone/filter_state.h"
#include <hh_ardrone/Map.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

	// Modify  = =
  ros::NodeHandle nh;
  std::string dronepose_channel;
  std::string nodeout_channel;

  ros::Subscriber subPose;
  ros::Publisher pubNodegen;

  float ddx,ddy,ddz,ddh;
  
  float image_stamp = 0;

  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);

	dronepose_channel = nh.resolveName("ardrone/predictedPose");
	nodeout_channel = nh.resolveName("hh_ardrone/houghLine");

	subPose = nh.subscribe(dronepose_channel,10, &ImageConverter::posCb, this);
	pubNodegen   = nh.advertise<hh_ardrone::Map>(nodeout_channel,1);
    

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }


  cv::Mat redFilter(const cv::Mat& src)
{
    assert(src.type() == CV_8UC3);
    cv::Mat redOnly;
    cv::inRange(src, cv::Scalar(0, 0, 0), cv::Scalar(25, 25, 25), redOnly);
	//cvInRangeS(imgHSV, cvScalar(170,160,60), cvScalar(180,256,256), imgThresh); 
    return redOnly;
}


void posCb(const tum_ardrone::filter_stateConstPtr pose)
{
	tum_ardrone::filter_state state = *pose;
	ddx = state.x;	  //Drone X
	ddy = state.y;	  //Drone Y
	ddz = state.z;	  //Drone Z
	ddh = state.yaw;  //Drone Heading
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    float drone_x,drone_y,drone_z,drone_h;
	
    drone_x = ddx;
    drone_y = ddy; 
    drone_z = ddz;
    drone_h = ddh;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());

    cv::Mat redOnly = redFilter(cv_ptr->image);

    cv::imshow("redOnly", redOnly);
    
    cv::Mat dst, cdst;
    cv::Canny(redOnly, dst, 100, 200, 3);
    cv::cvtColor(dst, cdst, CV_GRAY2BGR);

    std::vector<cv::Vec4i> lines;

    //cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
    cv::HoughLinesP(dst, lines, 1, CV_PI/2, 50, 50, 10 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
    	cv::Vec4i l = lines[i];
    	cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    	//ROS_INFO("Pilar no. %d x1 : %d, y1 : %d  x2 %d  y2 %d",i ,l[0],l[1],l[2],l[3] );
    	hh_ardrone::Map node;
	node.drone_x = drone_x;
	node.drone_y = drone_y;
	node.drone_z = drone_z;
	node.drone_h = drone_h;
	node.ImgStamp = image_stamp;
	node.Pillar_x1 = l[0];
 	node.Pillar_y1 = l[1];
	node.Pillar_x2 = l[2];
	node.Pillar_y2 = l[3];
	pubNodegen.publish(node);
     }

   image_stamp++;

   imshow("detected lines", cdst);


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

/* PREVIOUS version of houghLine.cpp code

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());

    //Hough Lines

	cv::Mat dst, cdst;
	cv::Canny(cv_ptr->image, dst, 50, 200, 3);
	cv::cvtColor(dst, cdst, CV_GRAY2BGR);

	std::vector<cv::Vec4i> lines;
	//cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
	//cv::HoughLinesP(dst, lines, 1, CV_PI/2, 50, 50, 10 );	//Horizontal
	cv::HoughLinesP(dst, lines, 1, CV_PI, 50, 50, 10 );	//Vertical
	for( size_t i = 0; i < lines.size(); i++ )
	{
	  cv::Vec4i l = lines[i];
	  cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
	}
	cv::imshow("detected lines", cdst);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

*/
