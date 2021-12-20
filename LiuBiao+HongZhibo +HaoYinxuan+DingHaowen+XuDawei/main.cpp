#include <iostream>
#include <opencv2/opencv.hpp>
#include "serialport.h"
#include "CRC_Check.h"
#include "sort_greenball.h"

using namespace std;
using namespace cv;

int main()
{
    int mode,sentry,base;
    char ttyUSB_path[] = "/dev/ttyUSB0";    //设置串口名称
    SerialPort port(ttyUSB_path);           //创建串口类对象
    port.initSerialPort();                  //串口初始化
    Mapdata data;

    VideoCapture cap(0);
    if(!cap.isOpened())
    {
        cout << " open failed! " << endl;
        return -1;
    }

    short num = 2;
    ball_sort sort_green;   //类的使用
    while (true)
    {
        Mat frame;
        cap >> frame;
        if(frame.empty())
        {
            cout << " frame over! " << endl;
            break;
        }

        short color_green = sort_green.green_ball(frame);
        cout << color_green << endl;
        
        namedWindow("dst", WINDOW_AUTOSIZE);
        imshow("dst", frame);

        if(color_green == 1)
        {
            data = {color_green};
            port.TransformData(data);
            port.send();
        }
        else 
        {
            data = {num};
            port.TransformData(data);
            port.send();
        }
        waitKey(1);
    }
    return 0;
}