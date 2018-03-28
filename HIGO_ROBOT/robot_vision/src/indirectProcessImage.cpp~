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
#include <sensor_msgs/RegionOfInterest.h>
#include "std_msgs/String.h"
#include <sstream>



using namespace std;
using namespace cv;



#define pi 3.14159265
#define radToDegree  57.2958


using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

Eigen::MatrixXf RTransform(4, 4);
Eigen::MatrixXf camPose(4, 1);



#define   IF_MEASURE 

Rect    preroimsgs_;
Rect    nowroimsgs_;
int     enSendDot=0;

const int imageWidth = 640;                          
const int imageHeight = 360;
Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     
Mat Rl, Rr, Pl, Pr, Q;              
Mat xyz;            

Point origin;        
Rect selection;      
bool selectObject = false; 

int blockSize = 0, uniquenessRatio = 0, numDisparities = 16;
//Ptr<StereoBM> bm = StereoBM::create(16, 9);
StereoBM bm;

 
/*
事先标定好的相机的参数
fx 0 cx
0 fy cy
0 0 1
*/
Mat cameraMatrixL = (Mat_<double>(3, 3) << 779.8856855348552, 0, 237.4199467459503,
0, 779.9889920700755, 193.102444487475,
0, 0, 1);
Mat distCoeffL = (Mat_<double>(5, 1) << 0.1946948859058712, -0.04091853710764933, 0.0003160116688853894, -0.03706066145318408, 0.7993636714615561);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 781.5336079917449, 0, 245.5048890133797,
0, 783.959841982888, 206.1181912600801,
0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << 0.2918417419654106, -1.773508449711521, 0.004601956295363975, -0.03078999663528584, 9.857115683235417);

Mat T = (Mat_<double>(3, 1) << -64.12248646570119,
-0.1153654690658362,
8.004752969467615);//T平移向量
Mat rec = (Mat_<double>(3, 1) << 0.03140106237508605,
-0.02182721194577655,
-0.002887291131761299);//rec旋转向量
Mat R;//R 旋转矩阵






geometry_msgs::Twist arm_pose_;	
Rect    roimsgs_;
Rect    roimsgs_1;
static  int arm_voice_en=0;

double pan_c_position=0.0;
double tilt_c_position=0.0;

Mat disp, disp8;





void D_H()
{
	
 	double L01=(-18.0)/100;
	double L12=(10.5)/100;
	double L23=36.5/100;
	double L34=3.0/100;
	double L45=5.0/100;
	double L56=4.0/100;
	double L67=5.2/100;
	double L78=2.3/100;
	double L89=3.0/100;
	double L910=0.5/100;



        double a1 =pan_c_position *radToDegree * pi /180;
	double a2 =tilt_c_position *radToDegree * pi /180;
	double a3 = 90 * pi /180;
	double a4 =-90 * pi /180;


	double s1 = sin(a1);
	double c1 = cos(a1);

	double s2 = sin(a2);
	double c2 = cos(a2);

	double s3 = sin(a3);
	double c3 = cos(a3);

	double s4 = sin(a4);
	double c4 = cos(a4);


	

    Eigen::MatrixXf  A01(4, 4), T01(4, 4);
    A01 << 1, 0, 0, 0,
	   0, 1, 0, 0, 
           0, 0, 1, -L01,
	   0, 0, 0, 1;
	T01 = A01;


    Eigen::MatrixXf  A12(4, 4), T02(4, 4);
    A12 << 1, 0, 0, -L12,
	   0, 1, 0, 0, 
           0, 0, 1, 0,
	   0, 0, 0, 1;
    T02 = T01 * A12;


    Eigen::MatrixXf A23(4, 4), T03(4, 4);
    A23 <<   1, 0, 0, 0,
	     0, 1, 0, 0,
             0, 0, 1, L23,
	     0, 0, 0, 1;
    T03 = T02 * A23;


    Eigen::MatrixXf  A34(4, 4), T04(4, 4);
    A34 <<   1, 0, 0, L34,
	     0, 1, 0, 0, 
             0, 0, 1, 0,
	     0, 0, 0, 1;          
    T04 = T03 * A34;


    Eigen::MatrixXf   A45(4, 4), T05(4, 4);
	A45 <<  1, 0, 0, 0, 
	        0, 1, 0, 0, 
	        0, 0, 1, L45,
	        0, 0, 0, 1;
    T05 = T04  * A45;


    Eigen::MatrixXf  A56(4, 4),R45(4,4),T06(4, 4);
    R45 <<  c1, -s1, 0, 0,
            s1, c1, 0, 0,
	     0, 0, 1, 0,
	     0, 0, 0, 1;
	         
	A56 <<  1, 0, 0, 0,
	        0, 1, 0, 0, 
	        0, 0, 1, L56,
	        0, 0, 0, 1;
	T06 = T05 * R45 * A56;


	Eigen::MatrixXf R56(4, 4), A67(4, 4), T07(4, 4);
    R56 <<  c2, 0, s2, 0,
	     0, 1, 0, 0, 
            -s2, 0, c2,0,
	      0, 0, 0, 1;
	        
	A67 <<  1, 0, 0, 0,
	         0, 1, 0, 0, 
	         0, 0, 1, L67, 
	         0, 0, 0, 1;
	T07 = T06 * R56 * A67;
	
    Eigen::MatrixXf  A78(4, 4), T08(4, 4);
    A78 <<  1, 0, 0, L78,
	        0, 1, 0, 0, 
            0, 0, 1, 0,
	        0, 0, 0, 1;
    T08 = T07 * A78;
	
	
    Eigen::MatrixXf  A89(4, 4), T09(4, 4);
	A89 <<  1, 0, 0, 0, 
            0, 1, 0, L89,
            0, 0, 1, 0,
	        0, 0, 0, 1;
    T09 = T08 *A89 ;
	
    Eigen::MatrixXf  A910(4, 4),T0P(4, 4),R1(4,4),R2(4,4);
    A910 <<  1, 0, 0, L910,
	        0, 1, 0, 0, 
            0, 0, 1, 0,
	        0, 0, 0, 1;
    R1    <<  c3, 0, s3, 0,
            0, 1, 0, 0,
            -s3, 0, c3,0,
             0, 0, 0, 1;
    R2 <<  c4, -s4, 0, 0,
            s4, c4, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    T0P = T09 *A910 * R1 * R2;
	
  	RTransform = T0P;
	
	//cout << RTransform << endl;
}

static void autochoice(Point temp)
{
        Eigen::MatrixXf targetPose(4, 1);

		
	D_H();
	camPose << xyz.at<Vec3f>(temp)[0] * 0.001, xyz.at<Vec3f>(temp)[1] * 0.001, xyz.at<Vec3f>(temp)[2] * 0.001, 1;


        if(camPose(2,0)!=160)
        {
	  cout << temp << "in world coordinate is: " << camPose << endl;
	  //cout << camPose << endl;
          targetPose=RTransform*camPose;
	  cout << temp << "in world coordinate targetPose is: " << targetPose << endl;
	  //cout << targetPose << endl;
		
	  arm_pose_.linear.x= targetPose(0);
 	  arm_pose_.linear.y= targetPose(1);
	  arm_pose_.linear.z= targetPose(2);

        } 


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


	bm(rectifyImageL, rectifyImageR, disp);
	//  bm->compute(rectifyImageL, rectifyImageR, disp);
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));
	reprojectImageTo3D(disp, xyz, Q, true); 
	xyz = xyz * 16;
        
       if(enSendDot==2)
       {
         enSendDot=0;
         rectangle(disp8, roimsgs_, Scalar(255, 255, 255), 1, 8, 0);
        //rectangle(disp8, roimsgs_1, Scalar(255, 255, 255), 1, 8, 0);
         Point p_t=Point(roimsgs_.x, roimsgs_.y);
         for(int i=0;i< roimsgs_.width;i++)
         {
            for(int j=0;j< roimsgs_.height;j++)
            {
                 
                // cout << p_t << "in world coordinate is: " << xyz.at<Vec3f>(p_t) << endl;
                
            }
         }
         autochoice(Point(roimsgs_.x, roimsgs_.y));
       }

       imshow("disparity", disp8);
}




static void onMouse(int event, int x, int y, int, void*)
{
       
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
		selection = Rect(x, y, 0, 0);
		selectObject = true;
                autochoice(origin);
		
		break;
	case EVENT_LBUTTONUP:    //Êó±ê×ó°ŽÅ¥ÊÍ·ÅµÄÊÂŒþ
		selectObject = false;
		arm_voice_en=2;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}


void panStateCallBack(const dynamixel_msgs::JointState& state)
{
    pan_c_position=state.current_pos;
    //cout<<"the pan pose is "<<  pan_c_position <<endl;
}

void tiltStateCallBack(const dynamixel_msgs::JointState& state)
{
    tilt_c_position=state.current_pos;
   // cout<<"the tilt pose is "<<  tilt_c_position <<endl;
}



void camshiftGetRoiCallBack(const sensor_msgs::RegionOfInterest& roimsgs )
{       
            roimsgs_.x=roimsgs.x_offset;
            roimsgs_.y=roimsgs.y_offset;
            roimsgs_.width=roimsgs.width;
            roimsgs_.height=roimsgs.height;

   //   cout<<"the x  is "<<  roimsgs.x_offset <<endl;
   //   cout<<"the y  is "<<  roimsgs.y_offset <<endl;
   //   cout<<"the width  is "<<  roimsgs.width <<endl;
   //   cout<<"the height  is "<<  roimsgs.height <<endl;
}

void camshiftGetRoiCallBack1(const sensor_msgs::RegionOfInterest& roimsgs )
{

      roimsgs_1.x=roimsgs.x_offset;
      roimsgs_1.y=roimsgs.y_offset;
      roimsgs_1.width=roimsgs.width;
      roimsgs_1.height=roimsgs.height;
   //   cout<<"the x  is "<<  roimsgs.x_offset <<endl;
   //   cout<<"the y  is "<<  roimsgs.y_offset <<endl;
   //   cout<<"the width  is "<<  roimsgs.width <<endl;
   //   cout<<"the height  is "<<  roimsgs.height <<endl;
}

void imageVoiceCb( const std_msgs::String& msg)
{
          if( msg.data =="启动机械臂" )
          {
              arm_voice_en=2;
          }
          //if( msg.data =="停止机械臂" )
          {
             // arm_voice_en=0;
          }
}

void staticRoiCb( const std_msgs::String& msg)
{
          if( msg.data =="12" )
          {
              enSendDot=2;
          }
          if( msg.data =="34" )
          {
              enSendDot=0;
          }
}


int main(int argc, char** argv)
{
	   ros::init(argc, argv, "image_processor");
	   ros::NodeHandle nh;
	   image_transport::ImageTransport it(nh);
	   //image_transport::Publisher pub = it.advertise("camera/image", 1);

           ros::Publisher   pub1=nh.advertise<geometry_msgs::Twist>("/arm/pick",1);
           ros::Subscriber  headPanStatesubscriber =  nh.subscribe("/head_pan_joint/state/", 1,  panStateCallBack);
           ros::Subscriber  headTiltStateSubscriber = nh.subscribe("/head_tilt_joint/state/", 1, tiltStateCallBack);
           ros::Subscriber  camshiftRoiSubscriber =   nh.subscribe("/roi/left", 1,               camshiftGetRoiCallBack);
           ros::Subscriber  camshiftRoiSubscriber1 =  nh.subscribe("/roi/right", 1,              camshiftGetRoiCallBack1);
           ros::Subscriber  pickVoiceSubscribe     =  nh.subscribe("/Rog_result",1,              imageVoiceCb); 
           ros::Subscriber  staticRoiSubscribe     =  nh.subscribe("/static/roi",1,              staticRoiCb); 


	   cv::Mat image,image1;
	   cv::Mat frame,frame1;


	   namedWindow("disparity", CV_WINDOW_AUTOSIZE);
	   createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
	   setMouseCallback("disparity", onMouse, 0);
	
	
           ros::Rate loop_rate(33);
	   while (nh.ok()) 
	   {
           image1=imread("/home/ros/gohi_ws/src/HIGO_ROBOT/robot_vision/data/imageL.jpg");
           image=imread( "/home/ros/gohi_ws/src/HIGO_ROBOT/robot_vision/data/imageR.jpg");

            if((image1.cols!=640)||(image1.rows!=360)||(image.cols!=640)||(image.rows!=360))
            {

                 image1=imread("/home/ros/gohi_ws/src/HIGO_ROBOT/robot_vision/data/back_imageL.jpg");
                 image=imread( "/home/ros/gohi_ws/src/HIGO_ROBOT/robot_vision/data/back_imageR.jpg");
            }

#ifdef IF_MEASURE

		Rodrigues(rec, R); //Rodrigues
		stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
			0, imageSize, &validROIL, &validROIR);
		initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
		initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);


		cvtColor(image, grayImageL, CV_BGR2GRAY);
		cvtColor(image1, grayImageR, CV_BGR2GRAY);

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


        if(arm_voice_en==2)
        {
           arm_voice_en=0;
           pub1.publish(arm_pose_);
        }

       char key = cvWaitKey(33);
       ros::spinOnce(); 
       }
  
        destroyWindow("left");
        destroyWindow("right");
        destroyWindow("disparity");
	return 0;
}
