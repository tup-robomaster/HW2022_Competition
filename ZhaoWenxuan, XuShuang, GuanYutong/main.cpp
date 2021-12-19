#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include<iostream>
#include "serialport.h"

using namespace cv;
using namespace std;

//摄像头和球放在无光纯暗的位置
int main()
{
	Mat video1, video2, hsvImage, HsvImage, imgHSV, hsvSplit, imgThresholded;
	Mapdata data;
	VideoCapture video(0);
	if(!video.isOpened())
	{
		cout<<"摄像头未打开"<<endl;
		return 0;
	}
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化
	while(1)
	{
		Mapdata sign={0};
		video.read(video1);
		imshow("original", video1);
		Rect rect1(0, 0, 200, 200);//取roi，防止分拣时白球后方的绿球干扰
		video1(rect1).copyTo(video1);
		Mat video2 = video1.clone();
		vector<Mat> hsvSplit;
		cvtColor(video1, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV									
		split(imgHSV, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, imgHSV);
		inRange(imgHSV, Scalar(38, 50, 200), Scalar(75, 170, 255), imgThresholded); //Threshold the image//Scalar(38, 50, 200), Scalar(75, 170, 255)				
		Mat element1 = getStructuringElement(MORPH_RECT, Size(15, 15));
		dilate(imgThresholded, imgThresholded, element1, Point(-1, -1), 3);
		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(imgThresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
		for (int j = 0; j < contours.size(); j++)//遍历所有轮廓
  		{
      		RotatedRect box = minAreaRect(contours[j]); //包含该轮廓所有点
      		Point2f vertex[4];
      		box.points(vertex);
      		for (int i = 0; i < 4; i++)
      		{
        		line(video1, vertex[i], vertex[(i + 1) % 4], Scalar(255, 0, 0), 4, LINE_AA); //画线
     		}
      		putText(video1, "target", vertex[0], FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 0));//打印字体
			sign={1};
		}
		port.TransformData(sign);
		port.send();
		imshow("haoye", video1);
		if(waitKey(100) == 27)
		break;
	}
    return 0;
}