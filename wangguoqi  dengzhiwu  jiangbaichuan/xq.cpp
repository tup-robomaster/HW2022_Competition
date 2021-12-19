#include <opencv2/opencv.hpp>
#include<iostream>
#include <vector>
using namespace std;
using namespace cv;
  
 Mat bgr;
 Mat hsv;
 int hmin=0, smin=0, vmin=0, hmax=0, smax=0, vmax=0;
 string windowName = "src";
 Mat dst;
 bool judge(double a)
  {
    if (a>650)return true;
    return false;
 }
 int main()
 {
     
     Mat srcImage;//= imread("1.jpg") ;
	 VideoCapture cap(0);
	 Mat cap1;
	 while(1)
	 {
	 cap>>cap1;
	 srcImage=cap1.clone();
     imshow(windowName, srcImage);
     bgr = srcImage;
     //颜色空间转换  
     cvtColor( bgr,hsv, COLOR_BGR2HSV);
     dst = Mat::zeros(hsv.size(), hsv.type());
      
     Mat mask;
     inRange(hsv, Scalar(hmin=25,smin= 17,vmin= 0), Scalar(hmax=90, smax=209,vmax=255), mask);
      
      for (int r = 0; r < hsv.rows; r++)
      {
          for (int c = 0; c < hsv.cols; c++)
          {
              if (mask.at<uchar>(r, c) == 255)
              {
                  dst.at<Vec3b>(r, c)[0] = hsv.at<Vec3b>(r, c)[0];
                  dst.at<Vec3b>(r, c)[1] = hsv.at<Vec3b>(r, c)[1];
                  dst.at<Vec3b>(r, c)[2] = hsv.at<Vec3b>(r, c)[2];
              }
          }
      }
     GaussianBlur(mask,mask,Size(3,3),0,0);
	 Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	 morphologyEx(mask, mask, MORPH_OPEN, element);
	 morphologyEx(mask, mask, MORPH_CLOSE, element);
     Mat task;
     Canny(mask,task,126,225,3);
     vector<vector<Point>>contours;
     vector<Vec4i>hierarchy;
     findContours(task,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));

     vector<vector<Point>>::const_iterator itc = contours.begin();
	//计算轮廓的面积  
	for (int i = 0; i < (int)contours.size(); i++)
	{
		double g_dConArea = fabs(contourArea(contours[i], true));
		//cout << "轮廓面积： " << g_dConArea << endl;
	}
 
	//遍历查找最大面积轮廓
	itc = contours.begin();
	double MaxArea = fabs(contourArea(*itc));
	for (itc = contours.begin()+1; itc != contours.end();)
	{
		double g_dConArea = fabs(contourArea(*itc));
 
		if (g_dConArea > MaxArea)
		{
			MaxArea = g_dConArea;	
			itc = contours.erase(itc-1);
			itc++;
		}
		else {
			itc = contours.erase(itc);
		}
	}
	
	//查看检测到的最大轮廓面积
	for (int i = 0; i < (int)contours.size(); i++)
	{
		double max_dConArea = fabs(contourArea(contours[i], true));
		//cout << "最大轮廓面积： " << max_dConArea << endl;
    judge(max_dConArea);
    cout<<judge(max_dConArea)<<endl;
	}
  
     imshow("task", task);
    imshow("dst", dst);
     waitKey(1);
     }
     return 0;
    }
 