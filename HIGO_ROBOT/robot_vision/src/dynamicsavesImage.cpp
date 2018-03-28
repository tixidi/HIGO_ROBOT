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


void onMouse(int Event, int x, int y, int flags, void* param)
{
	if (Event == CV_EVENT_LBUTTONDOWN)
	{
	

		imwrite("/home/ros/gohi_ws/src/HIGO_ROBOT/robot_vision/data/imageL.jpg", img1);
		imwrite("/home/ros/gohi_ws/src/HIGO_ROBOT/robot_vision/data/imageR.jpg", img);

	}


}
                       
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

void cacMoments(cv::Mat src)
{
	Mat srcGray;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	// 高斯滤波
	GaussianBlur(src, src, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
	// 灰度转换
	cvtColor(src, srcGray, CV_RGB2GRAY);

	// 轮廓边界检测
	findContours(srcGray, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	// 绘制边界
	//drawContours(resultMat, contours, -1, cvScalar(0, 0, 255));
	//printf("Number of contours: %d\n", (int)contours.size());
	// 计算轮廓矩
	vector<Moments> mu(contours.size());
	for (int i = 0; i < (int)contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}
	// 分析矩计算图像相关特征
	for (int i = 0; i < (int)contours.size(); i++)
	{
		// 面积 重心 边界轮廓长度
		int area = mu[i].m00;
		int cx = mu[i].m10 / mu[i].m00;
		int cy = mu[i].m01 / mu[i].m00;
		int perimeter = arcLength(contours.at(i), true);
		// 椭圆
		if (int(contours.at(i).size()) <= 5)
		{
			continue;
		}
		else
		{
			RotatedRect rRect = fitEllipse(contours.at(i));
			double orientation = rRect.angle;
			double orientation_rads = orientation*3.1416 / 180;
			double majorAxis = rRect.size.height > rRect.size.width ? rRect.size.height : rRect.size.width;
			double minorAxis = rRect.size.height > rRect.size.width ? rRect.size.width : rRect.size.height;
			// 圆形度 离心率 周长 直径
			double roundness = pow(perimeter, 2) / (2 * 3.1416*area);
			double eccentricity = sqrt(1 - pow(minorAxis / majorAxis, 2));
			double ratio = (minorAxis / majorAxis) * 100;
			double diameter = sqrt((4 * area) / 3.1416);
			// 输出相关特征信息
			//printf("Area: %d\n", area);
			//printf("Perimeter: %d\n", perimeter);
			//printf("Major Axis: %.1f\n", majorAxis);
			//printf("Minor Axis: %.1f\n", minorAxis);
			//printf("Orientation: %.1f\n", orientation);
			//printf("Roundness: %.1f\n", roundness);
			//printf("Eccentricity: %.1f\n", eccentricity);
			//printf("Ratio: %.1f\n", ratio);
			//printf("Diameter: %.1f\n", diameter);
			//printf("\n");
			// 绘制矩形及椭圆
			ellipse(src, rRect, cvScalar(0, 255, 0));
			Rect ret1 = boundingRect(Mat(contours[i]));//计算右上点集的边界矩形
			int avgX = (ret1.x + ret1.x + ret1.width) / 2; //运动物体的矩形的中点X位置
			int avgY = (ret1.y + ret1.y + ret1.height) / 2;//运动物体的矩形的中点Y位置
			cout << "start  x:" << ret1.x << "y:" << ret1.y << endl;
			cout << "size  x:" << ret1.width << "y:" << ret1.height << endl;

			cout << "center x:" << avgX << "y:" << avgY << endl;

			rectangle(src, ret1, cvScalar(0, 0, 255));
			//	rectangle(image, boundingRect(contours.at(i)), cvScalar(0, 0, 255));

			// 绘制相关坐标
			line(src, Point(0, 0), Point(src.cols, src.rows), cvScalar(0, 0, 255));

			line(src, Point(src.cols, 0), Point(0, src.rows), cvScalar(0, 0, 255));
			// 绘制起始线
			line(src, Point(cx, cy), Point((int)(cx + 30 * cos(orientation_rads)),
				(int)(cy + 30 * sin(orientation_rads))), cvScalar(255, 0, 0), 1);
			// 输出图像起始
			char pChar[100];
			sprintf(pChar, "Ori: %.0f", orientation);
			putText(src, pChar, Point(cx, cy), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255));
		}
	}
}

void camshiftGetRoiCallBack(const sensor_msgs::RegionOfInterest& roimsgs )
{

      roimsgs_.x=roimsgs.x_offset+30;
      roimsgs_.y=roimsgs.y_offset+30;
      roimsgs_.width=roimsgs.width+30;
      roimsgs_.height=roimsgs.height+30;
   //   cout<<"the x  is "<<  roimsgs.x_offset <<endl;
   //   cout<<"the y  is "<<  roimsgs.y_offset <<endl;
   //   cout<<"the width  is "<<  roimsgs.width <<endl;
   //   cout<<"the height  is "<<  roimsgs.height <<endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cv::namedWindow(INPUT);
  cv::namedWindow(OUTPUT);
  cv::namedWindow(INPUT1);
  cv::namedWindow(OUTPUT1);
  setMouseCallback(INPUT, onMouse, 0);

  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =  it.subscribe("/camera/image/image_raw/left", 1, imageCallback);
  image_transport::Subscriber sub1 = it.subscribe("/camera/image/image_raw/right", 1, imageCallback1);

  ros::Subscriber camshiftRoiSubscriber =   nh.subscribe("/roi", 1, camshiftGetRoiCallBack);

 


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




