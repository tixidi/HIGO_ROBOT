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



using namespace std;
using namespace cv;



#define pi 3.14159265


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
0 0  1
*/
Mat cameraMatrixL = (Mat_<double>(3, 3) << 716.9033286995689, 0, 295.5338349278387,
0, 717.4693823830561, 267.6692020476737,
0, 0, 1);
Mat distCoeffL = (Mat_<double>(5, 1) << 0.2761523666288087, -0.7221317137606657, 0.01280161145029237, -0.02253665827751586, -0.7672342343390787);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 714.9558022131841, 0, 276.1576212033052,
0, 718.5616280608135, 267.4592937022915,
0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << 0.3764346671046204, -3.108921936434877, 0.01579957397500981, -0.02796755836180179, 11.19279694946209);

Mat T = (Mat_<double>(3, 1) << -54.57875720761322,
-1.137902916716757,
4.227004250789617);//T平移向量
Mat rec = (Mat_<double>(3, 1) << 0.02244599268821824,
0.05124204558029886,
0.02398810648499701);//rec旋转向量
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
double L01=(8)/100;
double L12=(10.5)/100;
double L23=36.5/100;
double L34=3.0/100;
double L45=5.0/100;
double L56=4.0/100;
double L67=5.0/100;
double L78=2.3/100;
double L89=3.0/100;
double L910=0.5/100;


	//ž÷¹ØœÚœÇ¶È//pi Îª180¶È
double a0 = 0 * pi /180;
double a1 = 0 * pi /180;
double a2 = 0 * pi /180;
double a3 = 0 * pi /180;
double a4 = 0 * pi /180;
double a5 = 39 * pi /180;
double a6 = 76 * pi /180;
double a7 = 76 * pi /180;
double a8 =(-90) * pi /180;
double a9 =(-90) * pi /180;

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
	R67 <<  c7, 0,  s7, 0,
	         0, 1,   0, 0, 
	         -s7, 0, c7, 0, 
	         0, 0, 0, 1;
	         
	A78 <<  1, 0, 0, 0, 
	        0, 1, 0, 0, 
	        0, 0, 1, L78, 
	        0, 0, 0, 1;
	T08 = T07 * R67 * A78;
	
	
	
	
	Eigen::MatrixXf R78(4, 4), T09(4, 4);
	R78 <<   c8, -s8, 0, 0,
	         s8 , c8,  0,  0, 
	         0, 0,   1, 0, 
	         0, 0,   0, 1;
	T09 = T08 * R78;
	
	Eigen::MatrixXf R89(4, 4), T0P(4, 4);
	R89 <<   c9, -s9, 0, 0,
	         s9 , c9,  0,  0, 
	         0, 0,   1, 0, 
	         0, 0,   0, 1;
	T0P = T09 * R89;
	
  	RTransform = T01* R01 * A12 * R12 * A23 * R23 * A34 * R34 * A45 * R45 * A56 * R56 * A67 *R67 * A78 * R78;
        //RTransform = T0P*R87;
	
	cout << RTransform << endl;
}

/*****Á¢ÌåÆ¥Åä*****/
void stereo_match(int, void*)
{
	bm.state->roi1 = validROIL;
	bm.state->roi2 = validROIR;
	bm.state->preFilterCap = 31;

	bm.state->SADWindowSize = 2 * blockSize + 5;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = numDisparities * 16 + 16;//ÊÓ²îŽ°¿Ú£¬ŒŽ×îŽóÊÓ²îÖµÓë×îÐ¡ÊÓ²îÖµÖ®²î,Ž°¿ÚŽóÐ¡±ØÐëÊÇ16µÄÕûÊý±¶£¬intÐÍ;
	bm.state->textureThreshold = 10;
	bm.state->uniquenessRatio = uniquenessRatio;//uniquenessRatioÖ÷Òª¿ÉÒÔ·ÀÖ¹ÎóÆ¥Åä
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 32;
	bm.state->disp12MaxDiff = 1;

	Mat disp, disp8;
	bm(rectifyImageL, rectifyImageR, disp);
	//  bm->compute(rectifyImageL, rectifyImageR, disp);//ÊäÈëÍŒÏñ±ØÐëÎª»Ò¶ÈÍŒ
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//ŒÆËã³öµÄÊÓ²îÊÇCV_16SžñÊœ
	reprojectImageTo3D(disp, xyz, Q, true); //ÔÚÊµŒÊÇóŸàÀëÊ±£¬ReprojectTo3D³öÀŽµÄX / W, Y / W, Z / W¶ŒÒª³ËÒÔ16(Ò²ŸÍÊÇW³ýÒÔ16)£¬²ÅÄÜµÃµœÕýÈ·µÄÈýÎ¬×ø±êÐÅÏ¢¡£
	xyz = xyz * 16;
	imshow("disparity", disp8);
}


/*****ÃèÊö£ºÊó±ê²Ù×÷»Øµ÷*****/
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
	case EVENT_LBUTTONDOWN:   //Êó±ê×ó°ŽÅ¥°ŽÏÂµÄÊÂŒþ
		origin = Point(x, y);
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
		
		
		
		//pub1.publish(arm_pose_);
		

		break;
	case EVENT_LBUTTONUP:    //Êó±ê×ó°ŽÅ¥ÊÍ·ÅµÄÊÂŒþ
		selectObject = false;
		arm_flag=1;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  
  ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("chatter", 1);
  
  //ros::Publisher pub1= it.advertise<>("higoArm/pose", 1);
  
  VideoCapture cap(0);
  VideoCapture cap1(1);


  //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

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

	//namedWindow("left", 1);
	//namedWindow("right", 1);
	/*Á¢ÌåÆ¥Åä*/
	namedWindow("disparity", CV_WINDOW_AUTOSIZE);
	// ŽŽœšSADŽ°¿Ú Trackbar
	//createTrackbar("BlockSize:\n", "disparity", &blockSize, 8, stereo_match);
	// ŽŽœšÊÓ²îÎšÒ»ÐÔ°Ù·Ö±ÈŽ°¿Ú Trackbar
	//createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
	// ŽŽœšÊÓ²îŽ°¿Ú Trackbar
	createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
	//Êó±êÏìÓŠº¯ÊýsetMouseCallback(Ž°¿ÚÃû³Æ, Êó±ê»Øµ÷º¯Êý, Ž«žø»Øµ÷º¯ÊýµÄ²ÎÊý£¬Ò»°ãÈ¡0)
	setMouseCallback("disparity", onMouse, 0);
	
	

sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(33);
  while (1) 
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
  //---------------------------
  #ifdef IF_RANGE
		cvtColor(image, imageHSV, CV_BGR2HSV);
		cvtColor(image1, imageHSV1, CV_BGR2HSV);
	
		std::vector<cv::Mat> hsvChannels;
		std::vector<cv::Mat> hsvChannels1;

		cv::split(imageHSV, hsvChannels);
		cv::split(imageHSV1, hsvChannels1);

		// 0ÍšµÀÎªH·ÖÁ¿£¬1ÍšµÀÎªS·ÖÁ¿£¬2ÍšµÀÎªV·ÖÁ¿
		imageH = hsvChannels[0];
		imageS = hsvChannels[1];
		imageV = hsvChannels[2];

		imageH1 = hsvChannels1[0];
		imageS1 = hsvChannels1[1];
		imageV1 = hsvChannels1[2];


		int x = image.cols / 2 - 10;
		int y = image.rows / 2 - 10;

		centerRect = Rect(x, y, 20, 20);
		//ÖÐÐÄŸØÐÎÇøÓòHÉ«µ÷ºÍS±¥ºÍ¶È

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

			minMaxIdx(centerH, NULL, &lastCenterMaxH);//ŒÇÂŒµ±Ç°ÖÐÐÄÇøÓò×îŽó×îÐ¡Öµ
			minMaxIdx(centerS, NULL, &lastCenterMaxS);

			minMaxIdx(centerH1, NULL, &lastCenterMaxH1);//ŒÇÂŒµ±Ç°ÖÐÐÄÇøÓò×îŽó×îÐ¡Öµ
			minMaxIdx(centerS1, NULL, &lastCenterMaxS1);

			first = false;
		}
		else
		{
			nowCenterAvgH = mean(centerH)[0];
			nowCenterAvgS = mean(centerS)[0];

			nowCenterAvgH1 = mean(centerH1)[0];
			nowCenterAvgS1 = mean(centerS1)[0];

			//imshow("hsvImage", imageHSV);
			//imshow("srcImage", image);

			//imshow("hsvImage1", imageHSV1);
			//imshow("srcImage1", image1);


			//many diff is true
			if (!compareLast(nowCenterAvgH, lastCenterAvgH))//Èç¹û²îÒì±ÈœÏÐ¡£¬žüÐÂ£¬Èç¹û²îÒì±ÈœÏŽó£¬²»žüÐÂ
			{
				centerAvgH = nowCenterAvgH;
				centerAvgS = nowCenterAvgS;
			}
			if (!compareLast(nowCenterAvgH1, lastCenterAvgH1))//Èç¹û²îÒì±ÈœÏÐ¡£¬žüÐÂ£¬Èç¹û²îÒì±ÈœÏŽó£¬²»žüÐÂ
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

		// Ð¡ãÐÖµ¶ÔÔŽ»Ò¶ÈÍŒÏñœøÐÐãÐÖµ»¯²Ù×÷
		threshold(imgH, dstTempImage1, low_threshold, maxVal, cv::THRESH_BINARY);
		threshold(imgH1, dstTempImage11, low_threshold1, maxVal, cv::THRESH_BINARY);

		// ŽóãÐÖµ¶ÔÔŽ»Ò¶ÈÍŒÏñœøÐÐãÐÖµ»¯²Ù×÷
		threshold(imgH, dstTempImage2, high_threshold, maxVal, cv::THRESH_BINARY_INV);
		threshold(imgH1, dstTempImage21, high_threshold1, maxVal, cv::THRESH_BINARY_INV);

		// ŸØÕóÓëÔËËãµÃµœ¶þÖµ»¯œá¹û
		bitwise_and(dstTempImage1, dstTempImage2, dstImageH);
		bitwise_and(dstTempImage11, dstTempImage21, dstImageH1);


		low_threshold = centerAvgS - thresholdS;
		high_threshold = centerAvgS + thresholdS;

		low_threshold1 = centerAvgS1 - thresholdS1;
		high_threshold1 = centerAvgS1 + thresholdS1;

		// Ð¡ãÐÖµ¶ÔÔŽ»Ò¶ÈÍŒÏñœøÐÐãÐÖµ»¯²Ù×÷
		threshold(imgS, dstTempImage1, low_threshold, maxVal, cv::THRESH_BINARY);
		threshold(imgS1, dstTempImage11, low_threshold1, maxVal, cv::THRESH_BINARY);

		// ŽóãÐÖµ¶ÔÔŽ»Ò¶ÈÍŒÏñœøÐÐãÐÖµ»¯²Ù×÷
		threshold(imgS, dstTempImage2, high_threshold, maxVal, cv::THRESH_BINARY_INV);
		threshold(imgS1, dstTempImage21, high_threshold1, maxVal, cv::THRESH_BINARY_INV);
		// ŸØÕóÓëÔËËãµÃµœ¶þÖµ»¯œá¹û
		bitwise_and(dstTempImage1, dstTempImage2, dstImageS);
		bitwise_and(dstTempImage11, dstTempImage21, dstImageS1);

		addWeighted(dstImageH, 1.0, dstImageS, 1.0, 0.0, dstImage);
		addWeighted(dstImageH1, 1.0, dstImageS1, 1.0, 0.0, dstImage1);

		//imshow("resultMat", dstImage);
		//imshow("resultMat1", dstImage1);
		// ¶šÒåœá¹¹ÔªËØ
		cv::Mat element = cv::getStructuringElement(
			cv::MORPH_ELLIPSE, cv::Size(20, 20));
		// ÐÎÌ¬Ñ§¿ª²Ù×÷ 
		cv::Mat openedMat;
		cv::morphologyEx(dstImage, openedMat,
			cv::MORPH_OPEN, element);

		//canny±ßÔµŒì²â
		int edgeThresh = 50;
		Canny(openedMat, dstImage, edgeThresh, edgeThresh * 3, 3);
		//imshow("dstImage", dstImage);


		// œá¹¹ÔªËØ¶šÒå
		cv::Mat struElmen = getStructuringElement(cv::MORPH_RECT,
			cv::Size(3, 3), cv::Point(-1, -1));
		Mat resultMat;
		// ÐÎÌ¬Ñ§±Õ²Ù×÷     
		morphologyEx(dstImage, dstImage, cv::MORPH_CLOSE, struElmen);

		// ¶šÒåÂÖÀª²ÎÊý
		std::vector< std::vector<cv::Point> > contours;
		std::vector< std::vector<cv::Point> > resContours;
		std::vector< cv::Vec4i > hierarchy;
		// Á¬ÍšÓò²éÕÒ
		findContours(dstImage, contours, hierarchy,
			CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		// ÉžÑ¡Î±ÂÖÀª   
		for (size_t i = 0; i < contours.size(); i++)
		{
			if (fabs(contourArea(cv::Mat(contours[i]))) > 1000)
				resContours.push_back(contours[i]);
		}
		dstImage.setTo(0);
		// »æÖÆÂÖÀª
		drawContours(dstImage, resContours, -1,
			cv::Scalar(255, 0, 0), CV_FILLED);
		image.copyTo(resultMat, dstImage);

		imshow("resultMat", resultMat);
/*
                cv::Mat result;
		for (size_t n = 0; n != resContours.size(); ++n)
		{
			// È¥³ýžß¶È¿í¶È²»·ûºÏÌõŒþÇøÓò
			cv::Rect rect = cv::boundingRect(resContours[n]);
			int sub = cv::countNonZero(dstImage(rect));
			double ratio = double(sub) / rect.area();
			double wh_ratio = double(rect.width) / rect.height;
			if (ratio > 0.5 && wh_ratio >0.1 && wh_ratio < 10 &&
				rect.height > 12 && rect.width > 12)
			{
				result = image(rect);
				//cv::imshow("rect", result);
				//cv::waitKey(0);
			}
		}
*/

//// 定义结构元素
		cv::Mat element1 = cv::getStructuringElement(
			cv::MORPH_ELLIPSE, cv::Size(20, 20));
		// 形态学开操作 
		cv::Mat openedMat1;
		cv::morphologyEx(dstImage1, openedMat1,
			cv::MORPH_OPEN, element1);

		//canny边缘检测
		int edgeThresh1 = 50;
		Canny(openedMat1, dstImage1, edgeThresh1, edgeThresh1 * 3, 3);
		//	imshow("dstImage1", dstImage1);


		// 结构元素定义
		cv::Mat struElmen1 = getStructuringElement(cv::MORPH_RECT,
			cv::Size(3, 3), cv::Point(-1, -1));
		Mat resultMat1;
		// 形态学闭操作     
		morphologyEx(dstImage1, dstImage1, cv::MORPH_CLOSE, struElmen1);

		// 定义轮廓参数
		std::vector< std::vector<cv::Point> > contours1;
		std::vector< std::vector<cv::Point> > resContours1;
		std::vector< cv::Vec4i > hierarchy1;
		//// 连通域查找
		findContours(dstImage1, contours1, hierarchy1,
			CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		// 筛选伪轮廓   
		for (size_t i = 0; i < contours1.size(); i++)
		{
			if (fabs(contourArea(cv::Mat(contours1[i]))) > 1000)
				resContours1.push_back(contours1[i]);
		}
		dstImage1.setTo(0);
		// 绘制轮廓
		drawContours(dstImage1, resContours1, -1,
			cv::Scalar(255, 0, 0), CV_FILLED);
		image1.copyTo(resultMat1, dstImage1);
		//resultMat1.copyTo(segImageR);

		imshow("resultMat1", resultMat1);

#endif



#ifdef IF_MEASURE


//-------------------------------------------
		/* Á¢ÌåÐ£Õý */
		Rodrigues(rec, R); //Rodrigues±ä»»
		stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
			0, imageSize, &validROIL, &validROIR);
		initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
		initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

		/*¶ÁÈ¡ÍŒÆ¬*/
		//rgbImageL = imread("t1.jpg", CV_LOAD_IMAGE_COLOR);
		//cvtColor(image, grayImageL, CV_BGR2GRAY);
cvtColor(resultMat, grayImageL, CV_BGR2GRAY);
		//rgbImageR = imread("t2.jpg", CV_LOAD_IMAGE_COLOR);
		//cvtColor(image1, grayImageR, CV_BGR2GRAY);
cvtColor(resultMat1, grayImageR, CV_BGR2GRAY);
		//imshow("ImageL Before Rectify", grayImageL);
		//imshow("ImageR Before Rectify", grayImageR);

		/*Ÿ­¹ýremapÖ®ºó£¬×óÓÒÏà»úµÄÍŒÏñÒÑŸ­¹²Ãæ²¢ÇÒÐÐ¶Ô×ŒÁË*/
		remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
		remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

		/* °ÑÐ£Õýœá¹ûÏÔÊŸ³öÀŽ */
		Mat rgbRectifyImageL, rgbRectifyImageR;
		cvtColor(rectifyImageL, rgbRectifyImageL, CV_GRAY2BGR);  //Î±²ÊÉ«ÍŒ
		cvtColor(rectifyImageR, rgbRectifyImageR, CV_GRAY2BGR);

		//µ¥¶ÀÏÔÊŸ
		rectangle(rgbRectifyImageL, validROIL, Scalar(0, 0, 255), 3, 8);
		rectangle(rgbRectifyImageR, validROIR, Scalar(0, 0, 255), 3, 8);
		//imshow("ImageL After Rectify", rgbRectifyImageL);
		//imshow("ImageR After Rectify", rgbRectifyImageR);

		//ÏÔÊŸÔÚÍ¬Ò»ÕÅÍŒÉÏ
		Mat canvas;
		double sf;
		int w, h;
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);   //×¢ÒâÍšµÀ

		//×óÍŒÏñ»­µœ»­²ŒÉÏ
		Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //µÃµœ»­²ŒµÄÒ»²¿·Ö
		resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //°ÑÍŒÏñËõ·ÅµœžúcanvasPartÒ»ÑùŽóÐ¡
		Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //»ñµÃ±»œØÈ¡µÄÇøÓò
			cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
		//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //»­ÉÏÒ»žöŸØÐÎ
		// cout << "Painted ImageL" << endl;

		//ÓÒÍŒÏñ»­µœ»­²ŒÉÏ
		canvasPart = canvas(Rect(w, 0, w, h));                                      //»ñµÃ»­²ŒµÄÁíÒ»²¿·Ö
		resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
		Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
			cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
		//rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
		//cout << "Painted ImageR" << endl;

		//»­ÉÏ¶ÔÓŠµÄÏßÌõ
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
    pub.publish(msg);
    char key = cvWaitKey(33);
  }
  
  
  	cap.release();
	cap1.release();
	return 0;
}
