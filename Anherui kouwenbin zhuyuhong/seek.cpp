#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
using namespace std;
using namespace cv;
int iLowH = 28, iHighH = 71, iLowS = 57, iHighS = 219, iLowV = 0, iHighV = 255,p=0;
void color(Mat frame,int *m);
int main(int argc, char** argv)
{
    VideoCapture cap(0);
    if(!cap.isOpened())
    {
        cout<< "failed" <<endl;
        return 0;
    }
    while(1)
    {
        Mat frame;
        cap >> frame;
        if(frame.empty())
        {
            cout<< "frame off" <<endl;
            break;
        }
        color(frame, &p);//p是串口发送的东西
    }
    cap.release();
}
void color(Mat frame,int *m)
{
    vector<vector<Point>> contours;
	vector<Vec4i>hierchy;
    Mat imgHSV, imgThresholded, imgCanny, element;
    int cnt=0;
    resize(frame, frame, Size(320,240), 0, 0, INTER_AREA);
    cvtColor(frame, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, Scalar(iLowH,iLowS,iLowV), Scalar(iHighH,iHighS,iHighV), imgThresholded);
	element = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
    GaussianBlur(imgThresholded, imgThresholded, Size(5, 5), 3, 3);
    //预处理
    findContours(imgThresholded, contours, hierchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());//这里用RETR_EXTERNAL显示最外层轮廓
	 for(int i=0;i<contours.size();i++)  
    {  
        for(int j=0;j<contours[i].size();j++)  
        {   
            if(contours[i].size()>100) //contours[i].size()代表的是第i个轮廓上所有的像素点数  
            {
                cnt++;
                drawContours(frame,contours,i,Scalar(255),1,8,hierchy);  //绘制大于一定长度的轮廓
            }
            if(cnt>0)
            {
                *m=1;//找到球
                cout<<"1"<<endl;
            }
            else
            {
                *m=0;//没找到球
                cout<<"0"<<endl; 
            }   
        }
       
    } 
	imshow("11", imgThresholded);
    imshow("22", frame);   
}