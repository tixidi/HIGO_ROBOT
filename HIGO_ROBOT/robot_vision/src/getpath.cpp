#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
using namespace cv;
using namespace std;
Mat srcImage; Mat src_gray;
int thresh = 255;
int max_thresh = 255;
RNG rng(12345);



void thresh_callback(int, void*)
{

	Mat canny_output;
	vector<vector <Point> > contours;
	vector<Vec4i> hierarchy;
	// 用Canny算子检测边缘
	Canny(src_gray, canny_output, thresh, thresh * 2, 3);
	// 寻找轮廓
	findContours(canny_output, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	/// 绘出轮廓
	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		Scalar color = Scalar(rng.uniform(0, 255),
			rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		
		
	}

	cout << contours.size() << endl;
	//打印轮廓信息  
	std::cout << "共有外围轮廓：" << contours.size() << "条" << std::endl;
	std::vector<std::vector<Point> >::const_iterator itContours = contours.begin();
	for (; itContours != contours.end(); ++itContours)
	{
		std::cout << "每个轮廓的长度: " << itContours->size() << std::endl;
		
	}

	//除去太长或者太短的轮廓  
	int cmin = 100;
	int cmax = 1000;
	std::vector<std::vector<Point> >::iterator itc = contours.begin();
	
        while (itc != contours.end())
	{
		if (itc->size() < cmin || itc->size() > cmax)
			itc = contours.erase(itc);
                        //continue;
		else
			++itc;
	}

	Mat result(drawing.size(), CV_8U, Scalar(255));
	// 显示轮廓结果
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours", drawing);
	//将轮廓重绘于白板上  
	result.setTo(Scalar(255));
	drawContours(result, contours, -1, Scalar(0), 1);
	imshow("resulr", result);
	cout << result.channels() << endl;
	cout << result.rows << endl;
	cout << result.cols << endl;

        cv::imwrite("/home/ros/gohi_ws/src/HIGO_ROBOT/robot_blockly/scripts/map.jpg",result);




}


int main()
{
	cv::Mat srcImage =
		cv::imread("/home/ros/gohi_ws/src/HIGO_ROBOT/gohi_2dnav/map/chuanglian.pgm");
	if (!srcImage.data)
		return -1;
	// 转成灰度并平滑
	cvtColor(srcImage, src_gray, CV_BGR2GRAY);
	blur(src_gray, src_gray, Size(3, 3));

	// 创建窗体
	char* source_window = "srcImage";
	namedWindow(source_window, CV_WINDOW_AUTOSIZE);
	imshow(source_window, srcImage);
	// 滑动条控制canny阈值
	createTrackbar(" threth:", "srcImage", &thresh, max_thresh, thresh_callback);
	thresh_callback(0, 0);
        
	cv::waitKey(0);

	return(0);
}
