#include "serialport.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

void brightAdjust(Mat src,Mat dst,double dContrast,double dBright)
{
    int nVal;
    for(int nI = 0;nI<src.rows;nI++)
    {
        Vec3b* p1 = src.ptr<Vec3b>(nI);
        Vec3b* p2 = dst.ptr<Vec3b>(nI);
        for(int nJ =0;nJ<src.cols;nJ++)
        {
            for(int nK =0;nK<3;nK++)
            {
                nVal = (int)(dContrast *p1[nJ][nK] + dBright);
                if(nVal<0)
                    nVal=0;
                if(nVal>255)
                    nVal=255;
                p2[nJ][nK] = nVal;
            }
        }
    }
}

void Green_ball (const Mat &image,int a)
{ 
    Mat image1,image2,image3;

    int iLowH =35;
    int iHighH = 77;
    int iLowS = 43; 
    int iHighS = 255;
    int iLowV = 46;
    int iHighV = 255;

    cv::namedWindow("Control", WINDOW_AUTOSIZE); //创建一个窗口

    cv::createTrackbar("LowH", "Control", &iLowH,179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &iHighH, 179);
    cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &iHighS, 255);
    cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &iHighV, 255);

    Mat imgHSV;
    vector<Mat> hsvSplit;//创建Mat类型容器；
    cvtColor(image, imgHSV, COLOR_BGR2HSV); 
    
    split(imgHSV, hsvSplit);//通道分离
    equalizeHist(hsvSplit[1],hsvSplit[2]);//直方图均衡化
    merge(hsvSplit,imgHSV);//合并通道；

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), image1); 
    
    threshold(image1, image1, 30, 255, THRESH_BINARY);//二值化
    
    //开闭运算
    Mat element1 = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat element2 = getStructuringElement(MORPH_RECT, Size(5,5));
    morphologyEx(image1, image2, MORPH_OPEN, element1);
    morphologyEx(image2, image3, MORPH_CLOSE, element2);
    
    vector<vector<Point>>contours;
    vector<Vec4i> hierarchy;
    findContours(image3,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE,Point());

    vector<RotatedRect>minRects(contours.size());

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area>18000) 
            {
                a = 1;
            }
        //cout<<area<<endl;
        drawContours(image, contours, i, Scalar(255, 0, 255), 2, 8, Mat(), 0, Point());
    }
    imshow("原图",image);
    waitKey(1);
}

int main()
{
    VideoCapture capture(0);

    if ( !capture.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }

    int mode,sentry,base;
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化
    Mapdata data;
    Mat frame;

    while(1)
    {
        capture >> frame;
        int a;

        brightAdjust(frame,frame,1,-90);

        Green_ball (frame,a);
        if(a == 1)
        {
            data = {1};
            port.TransformData(data);
            port.send();
            // port.get_Mode(mode,sentry,base);
            waitKey(1);
        }
        else if(a != 1)
        {
            data = {0};
            port.TransformData(data);
            port.send();
            waitKey(1);
        }
    }
    return 0;
}
