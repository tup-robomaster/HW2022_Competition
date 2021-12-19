#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include "serialport.h"
#include "CRC_Check.h"
using namespace std;
using namespace cv;
int hmin = 30;
int hmax = 85;
int smin = 40;
int smax = 255;
int vmin = 45;
int vmax = 255;
int cnt;
bool flag;
Mapdata data;
vector <double> circlebox;
Mat hsv, src, canny_output, imghsv, mask, out;
vector <vector <Point> > contours;
vector <Vec4i> hierarchy;
//int thresh = 80;
void callback(int,void*){
    Mat dst;
    mask = hsv.clone();
    inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);
    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, out, MORPH_OPEN, element);
   // Canny(out, canny_output, thresh, thresh*2, 3);
    findContours(out, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    for(int i = 0; i < contours.size(); i++){
        Scalar color = Scalar(125, 43, 46);
        double area = contourArea(contours[i]);
        //亮球距离摄像头5cm左右是轮廓面积为一万，不亮球轮廓面积为1000多。
        if(area >= 1e3){
            if(circlebox.size() < 50){
                circlebox.push_back(area);
            }
            else{
            	flag = 1;
				data = {1};
                cnt++;
                cout << area << " " << cnt <<endl;
                circlebox.clear();
            }
        }
        //cout<< area<< endl;
        drawContours(src, contours, i, color, 2, 8, hierarchy, 0, Point());
    }
    imshow("find2",src);
}
int main(){
    VideoCapture cap(0);
    if(!cap.isOpened())
        return -1;
    int mode,sentry,base;
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化
    float x,y;
    namedWindow("find");
    createTrackbar("hmin", "find", &hmin, 180, callback);
    createTrackbar("hmax", "find", &hmax, 180, callback);
    createTrackbar("smin", "find", &smin, 255, callback);
    createTrackbar("smax", "find", &smax, 255, callback);
    createTrackbar("vmin", "find", &vmin, 255, callback);
    createTrackbar("vmax", "find", &vmax, 255, callback);
    while(1){
        cap >> src;
        hsv = imghsv.clone();
        cvtColor(src, hsv, COLOR_RGB2HSV);
        callback(0,0);
        imshow("find",src);
		if(!flag){
			data = {0};
		}
        port.TransformData(data);
        port.send();
		flag = 0;
        waitKey(1);
    }
    return 0;
}