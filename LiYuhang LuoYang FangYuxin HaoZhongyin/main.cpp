#include "serialport.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <dlfcn.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;
 
void brightAdjust(Mat img, Mat dst, double dContrast, double dBright)
{
    int nVal;
#pragma omp parallel for

   
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

 short green(Mat&img)
{
    int f=0;
    Mat dstimg,out,imghsv,output;
    brightAdjust(img,img,1,-80);
    cvtColor(img, imghsv, cv::COLOR_RGB2HSV);
    inRange(imghsv, Scalar(35, 43, 46), Scalar(100, 255, 255), dstimg);
    Mat element = getStructuringElement(MORPH_RECT, Size(15, 15)); 
    morphologyEx(dstimg, out, MORPH_OPEN, element);
    Mat element3 = getStructuringElement(MORPH_RECT,Size(7,7));
    erode(out,out,element3);
    Canny(out, output, 80, 255, 3 );
    vector<int> colorVec;
    vector<vector<Point>> Contours;
    vector<Vec4i> Hierarchy;
    vector<Point2d> points;
    findContours(output, Contours,RETR_EXTERNAL,CHAIN_APPROX_NONE, Point(0, 0));
    for (int i = 0; i < Contours.size(); i++)
    {              
        RotatedRect box = minAreaRect(Contours[i]);//最小矩形
        double area = contourArea(Contours[i]);//计算面积
        double length = arcLength(Contours[i], true);//计算周长
        
        Point2f vertex[4];
        box.points(vertex);
        if(area>=30000&&length>=500)//根据摄像头的位置设定周长和面积的限制
        { 
          int r=1;
          cout  << "length: " << length << endl;
          cout << "area: " << area<<endl;
          for (int i = 0; i < 4; i++)
            {
            line(img, vertex[i], vertex[(i + 1) % 4], Scalar(255, 0, 0), 4, LINE_AA);//将荧光小球框出

            }
          Point2f center = (vertex[0] + vertex[2]) / 2;
          circle(img,center,20,Scalar(43, 46, 255),FILLED);//标记中心点
          putText(img, "target", center, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 0));//标记target
          return r;
        }
         
    }      
            
    return f;
}    


 int main()
{
    Mat img;
    int mode,sentry,base;
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化
    Mapdata data;
    VideoCapture cap(2); 
    if(!cap.isOpened())  
      return -1;
    while (true)
    {
        cap>>img;
        if(img.empty())
                break;
        brightAdjust(img, img, 1,-20);//亮度调节
        short t;
        t=green(img);//荧光小球识别

        data = {t};
        port.TransformData(data);
        port.send();
        

        namedWindow("img", WINDOW_NORMAL);
        imshow("img",img);
        waitKey(50);

    }
    return 0;
}