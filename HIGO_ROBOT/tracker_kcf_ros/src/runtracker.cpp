#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include <math.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>

#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "kcftracker.hpp"
  


static const std::string RGB_WINDOW = "RGB Image window";
//static const std::string DEPTH_WINDOW = "DEPTH Image window";


using namespace cv;
using namespace std;
 // 初始化阈值参数

#define Min_distance_offset  	0.8
#define Max_linear_speed     	0.3
#define Min_linear_speed   	0.1
#define Min_distance       	1.3
#define low_threshold      	1.3
#define high_threshold     	1.8
#define Max_distance       	3.0
#define Max_rotation_speed 	0.75



float linear_speed = 0;
float rotation_speed = 0;

float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

float k_rotation_speed       = 0.004;
float h_rotation_speed_left  = 1.2;
float h_rotation_speed_right = 1.36;
 
int ERROR_OFFSET_X_left1  = 100;
int ERROR_OFFSET_X_left2  = 300;
int ERROR_OFFSET_X_right1 = 340;
int ERROR_OFFSET_X_right2 = 540;

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;


bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;
bool get_depth_flag   = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;
int kcf_voice_en=0;



// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

float dist_val[5] ;

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}

void cacMoments(cv::Mat src)
{
	Mat srcGray;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
        Mat temp;
        src.convertTo(temp, CV_8UC1, 1.0 );
	// 高斯滤波
	GaussianBlur(temp, temp, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
	// 轮廓边界检测
	findContours(temp, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	// 计算轮廓矩
	vector<Moments> mu(contours.size());
         int max_area=0;
	// 分析矩计算图像相关特征
	for (int i = 0; i < (int)contours.size(); i++)
	{

		if ((int(contours.at(i).size())<200))
		{
			continue;
		}
		else
		{
			// 绘制矩形及椭圆
			mu[i] = moments(contours[i], false);

                        if((boundingRect(contours.at(i)).width>100)&&(boundingRect(contours.at(i)).height)>240)
                        {

				rectangle(src, boundingRect(contours.at(i)), cvScalar(255, 255, 255));
                        }
			int cx = mu[i].m10 / mu[i].m00;
			int cy = mu[i].m01 / mu[i].m00;         
			//cout << boundingRect(contours.at(i)).x << endl;;
			//cout << boundingRect(contours.at(i)).y << endl;;
			//cout << boundingRect(contours.at(i)).width << endl;;
			//cout << boundingRect(contours.at(i)).height << endl;;
			//cout << cx << "  " << cy << endl;
                        selectRect.x = boundingRect(contours.at(i)).x;        
                        selectRect.y = boundingRect(contours.at(i)).y;
                        selectRect.width = boundingRect(contours.at(i)).width;   
                        selectRect.height = boundingRect(contours.at(i)).height/2;
		}
    
	}
	cv::imshow("result", src);
}

//OpenCV貌似也没有获取矩形中心点的功能，还是自己写一个
Point getCenterPoint(Rect rect)
{
    Point cpt;
    cpt.x = rect.x + cvRound(rect.width/2.0);
    cpt.y = rect.y + cvRound(rect.height/2.0);
    return cpt;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  
public:
  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Subscriber sub1;



  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,  &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image", 1,      &ImageConverter::depthCb, this);
    pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel", 1000);
    //publish /kcf/roi
    pub1= nh_.advertise<sensor_msgs::RegionOfInterest>("/kcf/roi", 1000);
    sub1= nh_.subscribe("/Rog_result",1,&ImageConverter::kcfVoiceCb, this);

    pub2= nh_.advertise<std_msgs::String>("/speak_string",1000);

    cv::namedWindow(RGB_WINDOW);
    //cv::namedWindow(DEPTH_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    //cv::destroyWindow(DEPTH_WINDOW);
  }
  void kcfVoiceCb( const std_msgs::String& msg)
  {
          if( msg.data =="初始跟踪" )
          {
              kcf_voice_en=1;
          }          
          if( msg.data =="开始跟踪" )
          {
              kcf_voice_en=2;
          }
          if( msg.data =="停止跟踪" )
          {
              kcf_voice_en=0;
          }
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image.copyTo(rgbimage);

   // cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

    if(bRenewROI)
    {
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    if(bBeginKCF)
    {
        result = tracker.update(rgbimage);
        cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
        enable_get_depth = true;
    }
    else
        cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);
    //OpenCV貌似也没有获取矩形中心点的功能，还是自己写一个
    //Point centerRect=getCenterPoint(result);

    //cout<<"image center  x is "<< rgbimage.cols<<"image center  y is "<<rgbimage.rows<<endl;
    // cout<<"center point  x is "<< centerRect.x<<"center point  y is "<<centerRect.y<<endl;

    cv::imshow(RGB_WINDOW, rgbimage);
    cv::waitKey(1);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
  	cv_bridge::CvImagePtr cv_ptr;
  	try
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
  		cv_ptr->image.copyTo(depthimage);

                get_depth_flag=1;
  	}
  	catch (cv_bridge::Exception& e)
  	{
  		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
  	}



       
       if(enable_get_depth)
       {

         dist_val[0] = depthimage.at<float>(result.y+result.height/3 , result.x+result.width/3) ;
         dist_val[1] = depthimage.at<float>(result.y+result.height/3 , result.x+2*result.width/3) ;
         dist_val[2] = depthimage.at<float>(result.y+2*result.height/3 , result.x+result.width/3) ;
         dist_val[3] = depthimage.at<float>(result.y+2*result.height/3 , result.x+2*result.width/3) ;
         dist_val[4] = depthimage.at<float>(result.y+result.height/2 , result.x+result.width/2) ;
      
         float distance = 0;
         int num_depth_points = 5;
         for(int i = 0; i < 5; i++)
         {
           if(dist_val[i] >Min_distance_offset && dist_val[i] < Max_distance)
             distance += dist_val[i];
           else
             num_depth_points--;
         }
         distance /= num_depth_points;
         cout<<distance<<endl;
//----------------------------------------------------------------------------------------------------
      //calculate rotation speed
      int center_x = result.x + result.width/2;

         std_msgs::String static_msg;  
         std::stringstream ss;
         if(center_x < ERROR_OFFSET_X_left1)         
          {
            rotation_speed =  Max_rotation_speed;
            if(kcf_voice_en==2){
            ss << "即将偏离目标" ;  
            static_msg.data = ss.str();  
            pub2.publish(static_msg);}
          }
          else if(center_x > ERROR_OFFSET_X_left1 && center_x < ERROR_OFFSET_X_left2)
          {
           rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left;
          }        
          else if(center_x > ERROR_OFFSET_X_right1 && center_x < ERROR_OFFSET_X_right2)        
          {
           rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right;
          }
          else if(center_x > ERROR_OFFSET_X_right2)
          {
            rotation_speed = -Max_rotation_speed;
            
            if(kcf_voice_en==2){
            ss << "即将偏离目标" ;  
            static_msg.data = ss.str();  
            pub2.publish(static_msg);}
          }        
          else  
          {
             rotation_speed = 0;
          }
       if( distance< Max_distance)
       {
          if(distance <Min_distance)  linear_speed = (-1.0)*distance * k_linear_speed + h_linear_speed;
          else                        linear_speed =        distance * k_linear_speed + h_linear_speed;
      }
      else 
      {
          linear_speed = 0;//在深度超过最大值的时候，停止机器人、语音提示丢失目标
          rotation_speed = 0;

          if(kcf_voice_en==2){
          ss << "丢失目标,我不走了" ;  
          static_msg.data = ss.str();  
          pub2.publish(static_msg);}

      }
    

      if(fabs(linear_speed) > Max_linear_speed)
        linear_speed = Max_linear_speed;

    }
  	cv::waitKey(1);
  }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kcf_tracker");
        ImageConverter ic;
  
	while(ros::ok())
	{
	  ros::spinOnce();


          if(kcf_voice_en==1)
          {
            while (!get_depth_flag);
       	    if(get_depth_flag==1)//初始跟踪
            {
               get_depth_flag=0;
               kcf_voice_en=0;
               Mat dep;

               cv::Mat dstTempImage1, dstTempImage2, dstImage;
               // 小阈值对源灰度图像进行阈值化操作
               cv::threshold( depthimage, dstTempImage1,low_threshold, Max_distance , cv::THRESH_BINARY );
               // 大阈值对源灰度图像进行阈值化操作
               cv::threshold( depthimage, dstTempImage2,high_threshold, Max_distance ,cv::THRESH_BINARY_INV );
               // 矩阵与运算得到二值化结果
               cv::bitwise_and( dstTempImage1, dstTempImage2, dep );
               imshow("depth", dep);
               cacMoments(dep);
               bBeginKCF = false;  
               bRenewROI = true;
            }

          }

           geometry_msgs::Twist twist;
           twist.linear.x = linear_speed;
           twist.linear.y = 0;
           twist.linear.z = 0;
           twist.angular.x = 0;
           twist.angular.y = 0;
           twist.angular.z = rotation_speed;
           if(kcf_voice_en==2) //开始跟踪
              ic.pub.publish(twist);

           sensor_msgs::RegionOfInterest regionInterest;
           regionInterest.x_offset=result.x;
           regionInterest.y_offset=result.y;
           regionInterest.height=result.height;
           regionInterest.width=result.width;

           ic.pub1.publish(regionInterest);

           if (cvWaitKey(33) == 'q')
             break;
	}

	return 0;
}

