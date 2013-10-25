 /**
 *  This file is part of hh_ardrone.
 *
 *  Copyright 2013 Yuantao Fan <fanyuantao@gmail.com> & Maytheewat Aramrattana <iamhere366@gmail.com>
 *
 *  hh_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with hh_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include "tum_ardrone/filter_state.h"
//#include <hh_ardrone/Map.h>
#include <hh_ardrone/map_info_msg.h>
#include <std_msgs/String.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

static const char WINDOW[] = "Image window";


class ImageConverter
{
private:
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
  
  int image_stamp = 0;
  int threshold = 25;	//Threshold ( pixel )

  //DRAWING PARAMETERS
  int w = 500;
  int width = 400;
  int half_width = width/2;
  int height = 650;
  int scale = 40;
  cv::Mat mapTraceImg = Mat::zeros( height, width, CV_8UC3 );

  //MOVED FROM PTAMINIT
  std::string ToPTAM_channel;
  std::string Space_command = "p space";
  ros::Publisher pubToPTAM;
  int key_frame1 = 1220;
  int key_frame2 = 1251;

  //HSV Params Defaults
  //inRange(redOnly, Scalar(100, 5, 13), Scalar(120, 209, 102), redOnly);
  //int minH = 80;
  //int minS = 33;
  //int minV = 34;
  //int maxH = 124;
  //int maxS = 179;
  //int maxV = 129;

  int minH = 79;
  int minS = 22;
  int minV = 13;
  int maxH = 145;
  int maxS = 178;
  int maxV = 247;


public:
  ImageConverter()
    : it_(nh_)
  {

    int KF1,KF2,T;
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 10, &ImageConverter::imageCb, this);

	dronepose_channel = nh.resolveName("ardrone/predictedPose");
	nodeout_channel = nh.resolveName("hh_ardrone/map_info");

	subPose = nh.subscribe(dronepose_channel,10, &ImageConverter::posCb, this);
	pubNodegen   = nh.advertise<hh_ardrone::map_info_msg>(nodeout_channel,1);

	ToPTAM_channel = nh.resolveName("tum_ardrone/com");
	pubToPTAM = nh.advertise<std_msgs::String>(ToPTAM_channel,50);

	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("KF1", KF1, 1220);
	private_node_handle_.param("KF2", KF2, 1251);
	private_node_handle_.param("T", T, 60);
    
	key_frame1 = KF1;
	key_frame2 = KF2;
	threshold = T;
	cout << "set initialize KF to " << KF1 << " and "<< KF2 << endl;
	cout << "set Threshold to " << T << " pixels" << endl;
/*
    cv::namedWindow(WINDOW);
    createTrackbar( "min H:", WINDOW, &minH, 180, NULL );
    createTrackbar( "min S:", WINDOW, &minS, 255, NULL );
    createTrackbar( "min V:", WINDOW, &minV, 255, NULL );
    createTrackbar( "max H:", WINDOW, &maxH, 180, NULL );
    createTrackbar( "max S:", WINDOW, &maxS, 255, NULL );
    createTrackbar( "max V:", WINDOW, &maxV, 255, NULL );
*/

  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }


  cv::Mat pillarFilter(const cv::Mat& src)
{
   assert(src.type() == CV_8UC3);
   cv::Mat redOnly;
   cvtColor(src, redOnly, CV_BGR2HSV);	//Convert to HSV
   //inRange(redOnly, Scalar(206, 2, 5), Scalar(238, 82, 40), redOnly); 
   //inRange(redOnly, Scalar(100, 5, 13), Scalar(120, 209, 102), redOnly);
   //inRange(redOnly, Scalar(100, 5, 13), Scalar(120, 209, 102), redOnly);
   inRange(redOnly, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), redOnly);

   cv::medianBlur( redOnly, redOnly, 9);
   
   //DRAW CONTOUR STUFF ?
   vector<vector<Point> > contours;
   findContours(redOnly.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

   Mat dst = Mat::zeros(src.size(), src.type());
   drawContours(dst, contours, -1, Scalar::all(255), CV_FILLED);

   return dst;
   //return redOnly;

/*
    assert(src.type() == CV_8UC3);
    cv::Mat redOnly;
    cv::inRange(src, cv::Scalar(0, 0, 0), cv::Scalar(25, 25, 25), redOnly);
	//cvInRangeS(imgHSV, cvScalar(170,160,60), cvScalar(180,256,256), imgThresh); 
    return redOnly;
*/

}


/**
 * Rotate an image
 */
void rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
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
    float px1,py1,px2,py2;
	
    //ROS_INFO("Got %d images",image_stamp);
    image_stamp++;
    //PTAM init
    if((image_stamp == key_frame1) || (image_stamp == key_frame2))
    {
	initPTAM();
    }
    

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

    //cv::imshow(WINDOW, cv_ptr->image);

    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());

    cv::Mat pillar = pillarFilter(cv_ptr->image);

    //cv::imshow("pillar", pillar);
    
    cv::Mat dst, cdst,rot;
    //cv::Canny(pillar, dst, 100, 200, 3);
    cv::Canny(pillar, dst, 120, 205, 3);
    //cv::cvtColor(dst, cdst, CV_GRAY2BGR);
	
    //ROTATE !!!!
    //rotate(dst, -drone_h, rot);
    Point2f src_center(dst.cols/2.0F, dst.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, drone_h, 1.0);
    warpAffine(dst, rot, rot_mat, dst.size());

    cv::cvtColor(rot, cdst, CV_GRAY2BGR);
    //imshow("Rotated Image",rot);

    std::vector<cv::Vec4i> lines;

    //cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
    //cv::HoughLinesP(dst, lines, 1, CV_PI/2, 50, 50, 10 );
    //cv::HoughLinesP(rot, lines, 1, CV_PI/2, 50, 50, 10 );
    //cv::HoughLinesP(rot, lines, 1, CV_PI/2, 50, 75, 20 );
    cv::HoughLinesP(rot, lines, 1, CV_PI/2, 50, 50, 20 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
	if(abs(l[0]-l[2]) > threshold && drone_z > 2.000)
        {
	  cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
          //cv::line( rot, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
	  //ROS_INFO("Pilar no. %d x1 : %d, y1 : %d  x2 %d  y2 %d",i ,l[0],l[1],l[2],l[3] );
	  hh_ardrone::map_info_msg node;
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

	  px1 = ((l[0]-320.0)*((2.0*drone_z)/734.3))+drone_x;
	  py1 = ((180.0-l[1])*((2.0*drone_z)/734.3))+drone_y;
	  px2 = ((l[2]-320.0)*((2.0*drone_z)/734.3))+drone_x;
	  py2 = ((180-l[3])*((2.0*drone_z)/734.3))+drone_y;

	  //Draw drone's trace
	  redCircle( mapTraceImg, Point( (drone_x*scale + half_width), (height-drone_y*scale)) );
	  MyLine( mapTraceImg, Point( (px1*scale+half_width), (height-py1*scale)), Point( (px2*scale+half_width), (height-py2*scale) ) ); 
	}     
      }

   //imshow("Lines",cdst);
   //image_stamp++;

   //imshow("Trace and Map", mapTraceImg);
   //cv::imshow("Trace and Map", mapTraceImg);	

  }

  //ADDED DRAWING FUNCIONS
  void MyLine( Mat img, Point start, Point end )
  {
    int thickness = 0.5;
    int lineType = 8;
    line( img,
	start,
	end,
	Scalar( 255, 0, 0 ),
	thickness,
	lineType );
  }
  void redCircle( Mat img, Point center )
  {
   int thickness = -1;
   int lineType = 8;

   circle( img,
         center,
         w/200.0,
         Scalar( 0, 0, 255 ),
         thickness,
         lineType );
  }
  void blueCirle( Mat img, Point center )
  {
   int thickness = -1;
   int lineType = 8;

   circle( img,
         center,
         w/200.0,
         Scalar( 255, 0, 0 ),
         thickness,
         lineType );
  }

  void initPTAM(){
	std_msgs::String s;
	s.data = Space_command.c_str();
	//pthread_mutex_lock(&tum_ardrone_CS);
	pubToPTAM.publish(s);
	//pthread_mutex_unlock(&tum_ardrone_CS);
	cout << "Init PTAM"<< endl;

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
