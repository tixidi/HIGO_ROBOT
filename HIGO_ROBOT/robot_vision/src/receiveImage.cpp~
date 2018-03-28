#include<ros/ros.h> 
#include<iostream> 
#include<cv_bridge/cv_bridge.h> 
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h> 
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>



static const std::string INPUT = "Input"; 
static const std::string OUTPUT = "Output"; 
static const std::string INPUT1 = "Input1"; 
static const std::string OUTPUT1 = "Output1";



using namespace cv;
using namespace std;
Mat img,img1;
Mat img_out,img_out1;
bool rightFLAG=0;
bool leftFLAG=0;


                       
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr cv_ptr; 
  try
  {

    leftFLAG=1;
    cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img=cv_ptr->image;



  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr cv_ptr1; 
  try
  {

    rightFLAG=1;
    cv_ptr1 =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img1=cv_ptr1->image;

 

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cv::namedWindow(INPUT);
  cv::namedWindow(OUTPUT);
  cv::namedWindow(INPUT1);
  cv::namedWindow(OUTPUT1);
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image/image_raw/left", 1, imageCallback);
  image_transport::Subscriber sub1 = it.subscribe("camera/image/image_raw/right", 1, imageCallback1);


  while (nh.ok()) 
  {
     if(leftFLAG)
     {
       leftFLAG=0;

       cv::cvtColor(img, img_out, CV_BGR2GRAY);  
       cv::imshow(INPUT, img);
       cv::imshow(OUTPUT, img_out);
        
     }
     if(rightFLAG)
     {
       rightFLAG=0;
  
       cv::cvtColor(img1, img_out1, CV_BGR2GRAY);  
       cv::imshow(INPUT1, img1);
       cv::imshow(OUTPUT1, img_out1);
  
        
     }

     char key = cvWaitKey(5);
     ros::spinOnce(); 
  }
 

   

  cv::destroyWindow(INPUT);
  cv::destroyWindow(OUTPUT);
  cv::destroyWindow(INPUT1);
  cv::destroyWindow(OUTPUT1);
}




