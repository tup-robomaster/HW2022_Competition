
#include<opencv2/opencv.hpp>
#include "serialport.h"
using namespace cv;
using namespace std;
int green_ball(Mat srcImage);
int white_ball(Mat srcImage);
void brightAdjust(Mat img, Mat dst, double dContrast, double dBright);
int hmin=37;//滑动条初始值
int hmax=107;
int smin=0;
int smax=114;
int vmin=46;
int vmax=255;
    int flag;
    Mapdata data;
  //  float x,y;
int main()
{
    int mode,sentry,base;
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化
    VideoCapture cap(2);
    if (!cap.isOpened())
    {
        return -1;
    }
    while (1)
    {    Mat src;
         cap >>src;
         if(src.empty())
         break;
         brightAdjust(src,src,1,-120);
        green_ball(src);
        white_ball(src);
        if(flag==1)
        {
        data = {1};
        port.TransformData(data);
        port.send();
        }
       if(flag==2)
      {
           data = {2};
        port.TransformData(data);
        port.send();
      }
        namedWindow("Contours Image",WINDOW_NORMAL);
	    imshow("Contours Image",src);
        waitKey(200);
    }
    return 0;
}

int green_ball(Mat srcImage)
{
    Mat gray, tchange;
    cvtColor(srcImage,gray,COLOR_BGR2HSV);
   // inRange(gray,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),tchange);
    inRange(gray,Scalar(30,45,60),Scalar(80,255,255),tchange);//二值化荧光
    Mat kernel=getStructuringElement(MORPH_RECT,Size(30,30));
    Mat t2change;
    morphologyEx(tchange,t2change,MORPH_OPEN,kernel);
    namedWindow("green",WINDOW_NORMAL);
    imshow("green",t2change);
    vector<vector<Point>>contours; 
    vector<Vec4i>hierarchy;
    findContours(t2change,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point(0,0));
    vector<RotatedRect> minRects(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {      double area =contourArea(contours[i]);
           if(area>2000)
          {flag=1;}
        minRects[i] = minAreaRect(contours[i]);
        Point2f rectPoints[4];
        minRects[i].points(rectPoints);
        for (int j = 0; j < 4; j++)
        {
            line(srcImage, rectPoints[j], rectPoints[(j+1)%4], Scalar(0,255,0), 2, 8, 0);
        }
    }
}
int white_ball(Mat srcImage)
{
//   namedWindow("测试窗口",WINDOW_AUTOSIZE);
//     createTrackbar("hmin","测试窗口",&hmin,180);//滑动条
//     createTrackbar("hmax","测试窗口",&hmax,180);
//     createTrackbar("smin","测试窗口",&smin,255);
//     createTrackbar("smax","测试窗口",&smax,255);
//     createTrackbar("vmin","测试窗口",&vmin,255);
//     createTrackbar("vmax","测试窗口",&vmax,255);
    Mat gray, tchange;
    cvtColor(srcImage,gray,COLOR_BGR2HSV);
    //inRange(gray,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),tchange);
    inRange(gray,Scalar(0,0,80),Scalar(180,51,255),tchange);//二值化白
    Mat kernel=getStructuringElement(MORPH_RECT,Size(30,30));
    Mat t2change;
    morphologyEx(tchange,t2change,MORPH_OPEN,kernel);
    namedWindow("white",WINDOW_NORMAL);
    imshow("white",t2change);
    vector<vector<Point>>contours; 
    vector<Vec4i>hierarchy;
    findContours(t2change,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point(0,0));
    vector<RotatedRect> minRects(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
         double area =contourArea(contours[i]);
           if(area>2000)
          {flag=2;}
        minRects[i] = minAreaRect(contours[i]);
        Point2f rectPoints[4];
        minRects[i].points(rectPoints);
        for (int j = 0; j < 4; j++)
        {
            line(srcImage, rectPoints[j], rectPoints[(j+1)%4], Scalar(0,0,255), 2, 8, 0);
        }
    }
   
}

void brightAdjust(Mat img, Mat dst, double dContrast, double dBright)
{
    int nVal;
   
    for (int nI = 0; nI<img.rows; nI++)
    {
        Vec3b* p1 = img.ptr<Vec3b>(nI); 
        Vec3b* p2 = dst.ptr<Vec3b>(nI); 
        for (int nJ = 0; nJ <img.cols; nJ++)
        {
            for (int nK = 0; nK < 3; nK++)  
            {
               //每个像素的每个通道的值都进行线性变换
                nVal = (int)(dContrast * p1[nJ][nK] + dBright); 
                if (nVal < 0)
                    nVal = 0;
                if (nVal > 255)
                    nVal = 255;
                p2[nJ][nK] = nVal; 
            }
        }
    }
}