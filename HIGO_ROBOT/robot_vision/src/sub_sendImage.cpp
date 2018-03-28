#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>

using namespace std;
using namespace cv;

cv::Mat image,image1;
cv::Mat frame,frame1;
cv::Mat resizeImage,resizeImage1;

int pic_count = 0;

int takeFLAG=0;
#define photo_NUM  500

void chatterCallback(const std_msgs::String::ConstPtr &msg)
 {

     ROS_INFO("I heard: [%s]", msg->data.c_str());  
     takeFLAG=1;

  }


int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("takePhotoFlag",1,chatterCallback); 
  
  VideoCapture cap(1);
  VideoCapture cap1(2);


  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image/image_raw/left", 1);
  image_transport::Publisher pub1 = it.advertise("camera/image/image_raw/right", 1);

 
 


  if (!cap.isOpened())	{
	cout << "error happened while open left cam " << endl;
	return 0;
   }
  if (!cap1.isOpened())	{
	cout << "error happened while open right cam " << endl;
	return 0;
  }
  
  namedWindow("left", 1);
  namedWindow("right", 1);
 

  ros::Rate loop_rate(500);
  while (nh.ok()) 
  {
    cap.read(frame);
    cap1.read(frame1);
    if (frame.empty())
    {
	cout << "no stream from  left cam " << endl;
	return 0;
    }
    if (frame1.empty())
    {
	cout << "no stream from  right cam " << endl;
	return 0;
    }
   
     frame.copyTo(image);
     frame1.copyTo(image1);
     imshow("left", image);
     imshow("right", image1);
     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
     sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();
  
     pub.publish(msg);
     pub1.publish(msg1);
     cout<<"send        "<<endl;    

 if(takeFLAG)
   {
     takeFLAG=0;

     pic_count++;
     string imageFileName;

     std::stringstream StrStm;
     StrStm << pic_count;
     StrStm >> imageFileName;
     imageFileName += ".ppm";

     
     resize(image1,resizeImage1,Size(128,64),0,0,CV_INTER_LINEAR);  
   
		
     if (pic_count >= photo_NUM) pic_count = 0;
      imwrite( "/home/ros/gohi_ws/src/robot_vision/data/imgname"+ imageFileName, resizeImage1 );
     cout<<"save_a_image"<<endl;	
   }
    

     char key = cvWaitKey(33);
     ros::spinOnce(); 
     //loop_rate.sleep();
  }
}
