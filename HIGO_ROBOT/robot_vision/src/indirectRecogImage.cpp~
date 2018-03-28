#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

#include <dynamixel_msgs/JointState.h>


using namespace std;
using namespace cv;



#define pi 3.14159265
#define radToDegree  57.2958


using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

Eigen::MatrixXf RTransform(4, 4);
Eigen::MatrixXf camPose(4, 1);


#define   IF_RANGE  //ÊÇ·ñÆ¥Åä
#define   IF_MEASURE //ÊÇ·ñ²âŸà



const int imageWidth = 640;                             //ÉãÏñÍ·µÄ·Ö±æÂÊ
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;//ÍŒÏñÐ£ÕýÖ®ºó£¬»á¶ÔÍŒÏñœøÐÐ²ÃŒô£¬ÕâÀïµÄvalidROIŸÍÊÇÖž²ÃŒôÖ®ºóµÄÇøÓò
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     //Ó³Éä±í
Mat Rl, Rr, Pl, Pr, Q;              //Ð£ÕýÐý×ªŸØÕóR£¬Í¶Ó°ŸØÕóP ÖØÍ¶Ó°ŸØÕóQ
Mat xyz;              //ÈýÎ¬×ø±ê

Point origin;         //Êó±ê°ŽÏÂµÄÆðÊŒµã
Rect selection;      //¶šÒåŸØÐÎÑ¡¿ò
bool selectObject = false;    //ÊÇ·ñÑ¡Ôñ¶ÔÏó

int blockSize = 0, uniquenessRatio = 0, numDisparities = 0;
//Ptr<StereoBM> bm = StereoBM::create(16, 9);
StereoBM bm;

/*
事先标定好的相机的参数
fx 0 cx
0 fy cy
0 0 1
*/
Mat cameraMatrixL = (Mat_<double>(3, 3) << 486.8024222034497, 0, 322.3627527629917,
0, 486.1221460071212, 247.4157190068541,
0, 0, 1);
Mat distCoeffL = (Mat_<double>(5, 1) << -0.3874488357988407, 0.3296752637511454, 0.0002518353198912691, 0.0002582480464933061, -0.4881895009885176);
Mat cameraMatrixR = (Mat_<double>(3, 3) << 490.953910210327, 0, 287.7769908302449,
0, 491.1866940845632, 212.6963220971602,
0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << -0.3426774889277792, -0.06873164879054711, 0.001964845460516886, -0.001665735910833493, 0.209198892534306);
Mat T = (Mat_<double>(3, 1) << -42.19636634474248,
-0.5098161159593785,
0.4878465559066463);//T平移向量
Mat rec = (Mat_<double>(3, 1) << 0.01390204617548014,
0.01803636495474853,
-0.02413264493544303);//rec旋转向量
Mat R;//R 旋转矩阵


//µØÃæãÐÖµ
double thresholdH;
double thresholdS;
int tempThresholdH = 50;
int tempThresholdS = 70;


//µ±Ç°ÖÐÐÄŸùÖµ
double nowCenterAvgH, centerAvgH;
double nowCenterAvgS, centerAvgS;


double lastCenterMaxH = 0.0;
double lastCenterMaxS = 0.0;


//ÉÏÒ»ŽÎÖÐÐÄŸùÖµ
double lastCenterAvgH;
double lastCenterAvgS;

//µØÃæãÐÖµ
double thresholdH1;
double thresholdS1;
int tempThresholdH1 = 50;
int tempThresholdS1 = 70;


//µ±Ç°ÖÐÐÄŸùÖµ
double nowCenterAvgH1, centerAvgH1;
double nowCenterAvgS1, centerAvgS1;


double lastCenterMaxH1 = 0.0;
double lastCenterMaxS1 = 0.0;


//ÉÏÒ»ŽÎÖÐÐÄŸùÖµ
double lastCenterAvgH1;
double lastCenterAvgS1;

//ÍŒÏñÔŽ
Mat srcImage;

Mat dstImage;
Mat dstImage1;

//HSVŒ°Æäž÷ÍšµÀÍŒÏñ
Mat imageHSV, imageH, imageS, imageV;
Mat imageHSV1, imageH1, imageS1, imageV1;


//ÖÐÐÄŸØÐÎÇøÓò
Rect centerRect;
// ÖÐÐÄŸØÐÎÆðÊŒµã
Point startPoint;
// ÖÐÐÄŸØÐÎÖÕÖ¹µã
Point endPoint;
// Íê³ÉËùÑ¡ÇøÓò±êÖŸÎ»
bool downFlag = false;
bool upFlag = false;
bool eventFlag = false;
//ÊÇ·ñÊÇµÚÒ»ŽÎÖÐÐÄµãÇøÓòŸùÖµ
bool first = true;
bool first1 = true;
//Ç°·œÊÇ·ñÓÐÕÏ°­Îï
bool obstacleFlag = false;

double maxH = 0.0;
double maxS = 0.0;
double maxH1 = 0.0;
double maxS1 = 0.0;


 geometry_msgs::Twist arm_pose_;	
static  int arm_flag=0;

double pan_c_position=0.0;
double tilt_c_position=0.0;

Point orientation;
char pChar[100];
void cacMoments(cv::Mat src);

void updateThreshold(int, void*)
{
	thresholdH = (maxH / lastCenterMaxH)*tempThresholdH;//žüžÄãÐÖµ
	thresholdS = (maxS / lastCenterMaxS)*tempThresholdS;//žüžÄãÐÖµ
}

void updateThreshold1(int, void*)
{
	thresholdH1 = (maxH1 / lastCenterMaxH1)*tempThresholdH1;//žüžÄãÐÖµ
	thresholdS1 = (maxS1 / lastCenterMaxS1)*tempThresholdS1;//žüžÄãÐÖµ
}

//ÓëÉÏÒ»Ö¡±ÈœÏÖÐÐÄŽŠHÉ«µ÷µÄŸùÖµ
bool compareLast(double h, double lh)
{
	return fabs(h - lh)>10;
}

void D_H()
{
	//²ÎÊý
double L01=(9-1.5+17.3)/100;
double L12=(36-7.6)/100;
double L23=3.0/100;
double L34=5.0/100;
double L45=4.0/100;
double L56=4.0/100;
double L67=6.0/100;
double L78=0.0/100;
double L89=0;
double L910=2.0/100;



	//ž÷¹ØœÚœÇ¶È//pi Îª180¶È
double a0 = 0 * pi /180;
double a1 = 0* pi /180;
double a2 = 0 * pi /180;
double a3 = 0 * pi /180;
double a4 = 0 * pi /180;
double a5 =pan_c_position *radToDegree * pi /180;
double a6 =tilt_c_position *radToDegree * pi /180;
double a7 = -90 * pi /180;
double a8 =0 * pi /180;
double a9 =90 * pi /180;

	double s0 = sin(a0);
	double c0 = cos(a0);

	double s1 = sin(a1);
	double c1 = cos(a1);

	double s2 = sin(a2);
	double c2 = cos(a2);

	double s3 = sin(a3);
	double c3 = cos(a3);

	double s4 = sin(a4);
	double c4 = cos(a4);

	double s5 = sin(a5);
	double c5 = cos(a5);

	double s6 = sin(a6);
	double c6 = cos(a6);
	
        double s7 = sin(a7);
	double c7 = cos(a7);

	
	double s8= sin(a8);
	double c8= cos(a8);
	
	double s9= sin(a9);
	double c9= cos(a9);
	
	//¹ØœÚ0 Aµã
	Eigen::MatrixXf A01(4, 4), T01(4, 4);
	A01 << 1, 0, 0, -L01, 
	       0, 1, 0, 0, 
	       0, 0, 1, 0, 
	       0, 0, 0, 1;
	T01 = A01;

	//¹ØœÚ2 Bµã
	Eigen::MatrixXf R01(4, 4), A12(4, 4), T02(4, 4);
	R01 << c1, 0, s1, 0, 
	       0, 1, 0, 0, 
	       -s1, 0, c1, 0, 
	       0, 0, 0, 1;
	       
	A12 << 1, 0, 0, 0, 
	       0, 1, 0, 0, 
	       0, 0, 1, L12, 
	       0, 0, 0, 1;
	T02 = T01 * R01 * A12;

	//¹ØœÚ3 Cµã
	Eigen::MatrixXf R12(4, 4), A23(4, 4), T03(4, 4);
	R12 <<   c2, 0, s2, 0,
	         0, 1, 0, 0, 
	         -s2, 0, c2, 0,
	          0, 0, 0, 1;
	          
	A23 <<   1, 0, 0, L23,
	         0, 1, 0, 0,
	         0, 0, 1, 0,
	         0, 0, 0, 1;
	T03 = T02 * R12 * A23;

	//¹ØœÚ4 Dµã
	Eigen::MatrixXf R23(4, 4), A34(4, 4), T04(4, 4);
	R23 <<  c3, 0, s3, 0,
	        0, 1, 0, 0, 
	        -s3, 0, c3, 0,
	         0, 0, 0, 1;
	         
	A34 <<   1, 0, 0, 0,
	         0, 1, 0, 0, 
	         0, 0, 1, L34,
	          0, 0, 0, 1;
	          
	T04 = T03 * R23 * A34;

	//¹ØœÚ5 Eµã
	Eigen::MatrixXf R34(4, 4), A45(4, 4), T05(4, 4);
	R34 <<  c4, 0, s4, 0, 
	        0, 1, 0, 0, 
	        -s4, 0, c4, 0,
	         0, 0, 0, 1;
	         
	A45 <<  1, 0, 0, 0, 
	        0, 1, 0, 0, 
	        0, 0, 1, L45,
	         0, 0, 0, 1;
	T05 = T04 * R34 * A45;

	//¹ØœÚ6 Fµã
	Eigen::MatrixXf R45(4, 4), A56(4, 4), T06(4, 4);
	R45 <<  c5, -s5, 0, 0, 
	        s5, c5, 0, 0, 
	        0, 0, 1, 0,
	        0, 0, 0, 1;
	         
	A56 <<  1, 0, 0, 0,
	        0, 1, 0, 0, 
	        0, 0, 1, L56,
	         0, 0, 0, 1;
	T06 = T05 * R45 * A56;
	//¹ØœÚ7 Pµã
	Eigen::MatrixXf R56(4, 4), A67(4, 4), T07(4, 4);
	R56 <<  c6, 0, s6, 0,
	        0, 1, 0, 0, 
	        -s6, 0, c6,0,
	         0, 0, 0, 1;
	        
	A67 <<  1, 0, 0, 0,
	         0, 1, 0, 0, 
	         0, 0, 1, L67, 
	         0, 0, 0, 1;
	T07 = T06 * R56 * A67;
	
	Eigen::MatrixXf R67(4, 4), A78(4, 4), T08(4, 4);
	R67 <<  1, 0,  0, 0,
	         0, c7, -s7, 0, 
	         0, s7, c7, 0, 
	         0, 0, 0, 1;
	         
	A78 <<  1, 0, 0, 0, 
	        0, 1, 0, 0, 
	        0, 0, 1, L78, 
	        0, 0, 0, 1;
	T08 = T07 * R67 * A78;
	
	
	
	
	Eigen::MatrixXf R78(4, 4), A89(4, 4), T09(4, 4);
	R78 <<  c8, 0, s8, 0,
	        0, 1, 0, 0, 
	        -s8, 0, c8,0,
	         0, 0, 0, 1;

	A89 <<  1, 0, 0, 0, 
	        0, 1, 0, 0, 
	        0, 0, 1, L89, 
	        0, 0, 0, 1;
	T09 = T08 * R78 *A89 ;
	
	Eigen::MatrixXf R89(4, 4), A910(4, 4),T0P(4, 4);
	R89 <<   c9, 0, s9, 0,
	        0, 1, 0, 0, 
	        -s9, 0, c9,0,
	         0, 0, 0, 1;
	A910 <<  1, 0, 0, 0, 
	        0, 1, 0, 0, 
	        0, 0, 1, L910, 
	        0, 0, 0, 1;
	T0P = T09 * R89*A910;
	
  	RTransform = T0P;
	
	cout << RTransform << endl;
}

void stereo_match(int, void*)
{
	bm.state->roi1 = validROIL;
	bm.state->roi2 = validROIR;
	bm.state->preFilterCap = 31;

	bm.state->SADWindowSize = 2 * blockSize + 5;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = numDisparities * 16 + 16;
	bm.state->textureThreshold = 10;
	bm.state->uniquenessRatio = uniquenessRatio;
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 32;
	bm.state->disp12MaxDiff = 1;

	Mat disp, disp8;
	bm(rectifyImageL, rectifyImageR, disp);
	//  bm->compute(rectifyImageL, rectifyImageR, disp);
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));
	reprojectImageTo3D(disp, xyz, Q, true); 
	xyz = xyz * 16;
	sprintf(pChar, "Ori: %d %d", orientation.x, orientation.y);
	putText(disp8, pChar, orientation, FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255));
	imshow("disparity", disp8);
}


static void onMouse(int event, int x, int y, int, void*)
{
        Eigen::MatrixXf targetPose(4, 1);
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:  
		origin = Point(x, y);
                orientation = origin;
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
		
		D_H();
		camPose << xyz.at<Vec3f>(origin)[0] * 0.001, xyz.at<Vec3f>(origin)[1] * 0.001, xyz.at<Vec3f>(origin)[2] * 0.001, 1;
		cout << camPose << endl;;
                targetPose=RTransform*camPose;
		cout << targetPose << endl;
		
		arm_pose_.linear.x= targetPose(0);
		arm_pose_.linear.y= targetPose(1);
		arm_pose_.linear.z= targetPose(2);
		
		break;
	case EVENT_LBUTTONUP:    //Êó±ê×ó°ŽÅ¥ÊÍ·ÅµÄÊÂŒþ
		selectObject = false;
		arm_flag=1;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}


void panStateCallBack(const dynamixel_msgs::JointState& state)
{
    //ROS_WARN_STREAM("Keep pan State.");
    pan_c_position=state.current_pos;
    cout<<"the pan pose is "<<  pan_c_position <<endl;
}

void tiltStateCallBack(const dynamixel_msgs::JointState& state)
{
   //ROS_WARN_STREAM("Keep tilt State.");
    tilt_c_position=state.current_pos;
    cout<<"the tilt pose is "<<  tilt_c_position <<endl;
}


void cacseg(cv::Mat& src)
{
	// 定义结构元素
	cv::Mat element = cv::getStructuringElement(
		cv::MORPH_ELLIPSE, cv::Size(20, 20));
	// 形态学开操作 
	cv::Mat openedMat;
	cv::morphologyEx(src, openedMat,
		cv::MORPH_OPEN, element);

	//canny边缘检测
	int edgeThresh = 50;
	Canny(openedMat, src, edgeThresh, edgeThresh * 3, 3);

	// 结构元素定义
	cv::Mat struElmen = getStructuringElement(cv::MORPH_RECT,
		cv::Size(3, 3), cv::Point(-1, -1));

	// 形态学闭操作     
	morphologyEx(src, src, cv::MORPH_CLOSE, struElmen);

	// 定义轮廓参数
	std::vector< std::vector<cv::Point> > contours;
	std::vector< std::vector<cv::Point> > resContours;
	std::vector< cv::Vec4i > hierarchy;
	// 连通域查找
	findContours(src, contours, hierarchy,
		CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	// 筛选伪轮廓   
	for (size_t i = 0; i < contours.size(); i++)
	{
		if (fabs(contourArea(cv::Mat(contours[i]))) > 1000)
			resContours.push_back(contours[i]);
	}
	src.setTo(0);
	// 绘制轮廓
	drawContours(src, resContours, -1,cv::Scalar(255, 0, 0), CV_FILLED);
	
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


int main(int argc, char** argv)
{
	   ros::init(argc, argv, "image_processor");
	   ros::NodeHandle nh;
	   image_transport::ImageTransport it(nh);
	   //image_transport::Publisher pub = it.advertise("camera/image", 1);
	  
           //--------------------------------------------------

           ros::Publisher  pub1=nh.advertise<geometry_msgs::Twist>("/chatter",1);
           ros::Subscriber headPanStatesubscriber =  nh.subscribe("/head_pan_joint/state/", 1, panStateCallBack);
           ros::Subscriber headTiltStateSubscriber = nh.subscribe("/head_tilt_joint/state/", 1, tiltStateCallBack);
 
	   VideoCapture cap(0);
           VideoCapture cap1(1);

	   cv::Mat image,image1;
	   cv::Mat frame,frame1;
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
	
	   namedWindow("disparity", CV_WINDOW_AUTOSIZE);
	
	   createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
	
	   setMouseCallback("disparity", onMouse, 0);
	
	
           ros::Rate loop_rate(33);
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
   
            frame.copyTo(image1);
            frame1.copyTo(image);
//
            imshow("left", image1);
            imshow("right", image);
//-----------------------------------------

#ifdef IF_RANGE
		cvtColor(image, imageHSV, CV_BGR2HSV);
		cvtColor(image1, imageHSV1, CV_BGR2HSV);
		// 分离HSV各个通道
		std::vector<cv::Mat> hsvChannels;
		std::vector<cv::Mat> hsvChannels1;

		cv::split(imageHSV, hsvChannels);
		cv::split(imageHSV1, hsvChannels1);

		// 0通道为H分量，1通道为S分量，2通道为V分量
		imageH = hsvChannels[0];
		imageS = hsvChannels[1];
		imageV = hsvChannels[2];

		imageH1 = hsvChannels1[0];
		imageS1 = hsvChannels1[1];
		imageV1 = hsvChannels1[2];


		int x = image.cols / 2 - 10;
		int y = image.rows / 2 - 10;

		centerRect = Rect(x, y, 20, 20);
		//中心矩形区域H色调和S饱和度

		Mat centerH(imageH, centerRect);
		Mat centerS(imageS, centerRect);
		Mat centerH1(imageH1, centerRect);
		Mat centerS1(imageS1, centerRect);

		rectangle(image, centerRect, Scalar(255, 0, 0), 1, 8, 0);
		rectangle(image1, centerRect, Scalar(255, 0, 0), 1, 8, 0);

		if (first)
		{
			lastCenterAvgH = mean(centerH)[0];
			lastCenterAvgS = mean(centerS)[0];

			lastCenterAvgH1 = mean(centerH1)[0];
			lastCenterAvgS1 = mean(centerS1)[0];

			minMaxIdx(centerH, NULL, &lastCenterMaxH);//记录当前中心区域最大最小值
			minMaxIdx(centerS, NULL, &lastCenterMaxS);

			minMaxIdx(centerH1, NULL, &lastCenterMaxH1);//记录当前中心区域最大最小值
			minMaxIdx(centerS1, NULL, &lastCenterMaxS1);

			first = false;
		}
		else
		{
			nowCenterAvgH = mean(centerH)[0];
			nowCenterAvgS = mean(centerS)[0];

			nowCenterAvgH1 = mean(centerH1)[0];
			nowCenterAvgS1 = mean(centerS1)[0];

			//many diff is true
			if (!compareLast(nowCenterAvgH, lastCenterAvgH))//如果差异比较小，更新，如果差异比较大，不更新
			{
				centerAvgH = nowCenterAvgH;
				centerAvgS = nowCenterAvgS;
			}
			if (!compareLast(nowCenterAvgH1, lastCenterAvgH1))//如果差异比较小，更新，如果差异比较大，不更新
			{
				centerAvgH1 = nowCenterAvgH1;
				centerAvgS1 = nowCenterAvgS1;
			}

			lastCenterAvgH = nowCenterAvgH;
			lastCenterAvgS = nowCenterAvgS;

			lastCenterAvgH1 = nowCenterAvgH1;
			lastCenterAvgS1 = nowCenterAvgS1;

			minMaxIdx(centerH, NULL, &maxH);
			minMaxIdx(centerS, NULL, &maxS);

			minMaxIdx(centerH1, NULL, &maxH1);
			minMaxIdx(centerS1, NULL, &maxS1);

			updateThreshold(0, 0);
			updateThreshold1(0, 0);


			lastCenterMaxH = maxH;
			lastCenterMaxS = maxS;
			lastCenterMaxH1 = maxH1;
			lastCenterMaxS1 = maxS1;

		}

		int maxVal = 255;
		Mat imgH, imgS;
		Mat imgH1, imgS1;

		double low_threshold = 0.0;
		double high_threshold = 0.0;
		double low_threshold1 = 0.0;
		double high_threshold1 = 0.0;


		blur(imageH, imgH, cv::Size(5, 5), cv::Point(-1, -1));//(5,5)
		blur(imageH1, imgH1, cv::Size(5, 5), cv::Point(-1, -1));//(5,5)
		blur(imageS, imgS, cv::Size(5, 5), cv::Point(-1, -1));//(5,5)
		blur(imageS1, imgS1, cv::Size(5, 5), cv::Point(-1, -1));//(5,5)

		low_threshold = centerAvgH - thresholdH;
		high_threshold = centerAvgH + thresholdH;
		low_threshold1 = centerAvgH1 - thresholdH1;
		high_threshold1 = centerAvgH1 + thresholdH1;

		Mat dstTempImage1, dstTempImage2, dstImageH, dstImageS, dstImage;
		Mat dstTempImage11, dstTempImage21, dstImageH1, dstImageS1, dstImage1;

		// 小阈值对源灰度图像进行阈值化操作
		threshold(imgH, dstTempImage1, low_threshold, maxVal, cv::THRESH_BINARY);
		threshold(imgH1, dstTempImage11, low_threshold1, maxVal, cv::THRESH_BINARY);

		// 大阈值对源灰度图像进行阈值化操作
		threshold(imgH, dstTempImage2, high_threshold, maxVal, cv::THRESH_BINARY_INV);
		threshold(imgH1, dstTempImage21, high_threshold1, maxVal, cv::THRESH_BINARY_INV);

		// 矩阵与运算得到二值化结果
		bitwise_and(dstTempImage1, dstTempImage2, dstImageH);
		bitwise_and(dstTempImage11, dstTempImage21, dstImageH1);


		low_threshold = centerAvgS - thresholdS;
		high_threshold = centerAvgS + thresholdS;

		low_threshold1 = centerAvgS1 - thresholdS1;
	    high_threshold1 = centerAvgS1 + thresholdS1;

		// 小阈值对源灰度图像进行阈值化操作
		threshold(imgS, dstTempImage1, low_threshold, maxVal, cv::THRESH_BINARY);
		threshold(imgS1, dstTempImage11, low_threshold1, maxVal, cv::THRESH_BINARY);

		// 大阈值对源灰度图像进行阈值化操作
		threshold(imgS, dstTempImage2, high_threshold, maxVal, cv::THRESH_BINARY_INV);
		threshold(imgS1, dstTempImage21, high_threshold1, maxVal, cv::THRESH_BINARY_INV);
		// 矩阵与运算得到二值化结果
		bitwise_and(dstTempImage1, dstTempImage2, dstImageS);
		bitwise_and(dstTempImage11, dstTempImage21, dstImageS1);

		addWeighted(dstImageH, 1.0, dstImageS, 1.0, 0.0, dstImage);
		addWeighted(dstImageH1, 1.0, dstImageS1, 1.0, 0.0, dstImage1);

		

		Mat resultMat;
		Mat resultMat1;


		cacseg(dstImage);
		image.copyTo(resultMat, dstImage);
		Mat temp = resultMat.clone();
		cacMoments(temp);
		temp = temp.clone();
		imshow("siteMat", temp);



		cacseg(dstImage1);
		image1.copyTo(resultMat1, dstImage1);
		Mat temp1 = resultMat1.clone();
		cacMoments(temp1);
		temp1 = temp1.clone();
		imshow("siteMat1", temp1);
#endif


#ifdef IF_MEASURE

		Rodrigues(rec, R); //Rodrigues
		stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
			0, imageSize, &validROIL, &validROIR);
		initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
		initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);


		/*读取图片*/
		//rgbImageL = imread("t1.jpg", CV_LOAD_IMAGE_COLOR);
		//cvtColor(image, grayImageL, CV_BGR2GRAY);
		//rgbImageR = imread("t2.jpg", CV_LOAD_IMAGE_COLOR);
		//cvtColor(image1, grayImageR, CV_BGR2GRAY);
		cvtColor(resultMat, grayImageL, CV_BGR2GRAY);
		cvtColor(resultMat1, grayImageR, CV_BGR2GRAY);


		//imshow("ImageL Before Rectify", grayImageL);
		//imshow("ImageR Before Rectify", grayImageR);


		remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
		remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);


		Mat rgbRectifyImageL, rgbRectifyImageR;
		cvtColor(rectifyImageL, rgbRectifyImageL, CV_GRAY2BGR);  
		cvtColor(rectifyImageR, rgbRectifyImageR, CV_GRAY2BGR);


		rectangle(rgbRectifyImageL, validROIL, Scalar(0, 0, 255), 3, 8);
		rectangle(rgbRectifyImageR, validROIR, Scalar(0, 0, 255), 3, 8);
		//imshow("ImageL After Rectify", rgbRectifyImageL);
		//imshow("ImageR After Rectify", rgbRectifyImageR);


		Mat canvas;
		double sf;
		int w, h;
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);  


		Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                               
		resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);   
		Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                
			cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
		//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                    
		// cout << "Painted ImageL" << endl;


		canvasPart = canvas(Rect(w, 0, w, h));                                      
		resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
		Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
			cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
		//rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
		//cout << "Painted ImageR" << endl;


		for (int i = 0; i < canvas.rows; i += 16)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		stereo_match(0, 0);
#endif

    if(arm_flag==1)
    {
      arm_flag=0;
      pub1.publish(arm_pose_);
    }
   // pub.publish(msg);
    char key = cvWaitKey(33);
    ros::spinOnce(); 
  }
  
        destroyWindow("left");
        destroyWindow("right");
        destroyWindow("disparity");
	return 0;
}
