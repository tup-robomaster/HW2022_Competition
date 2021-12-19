#include <iostream>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


void getContours(Mat imgDil, Mat img)
{
    vector<vector<Point>>contours;
    vector<Vec4i>hierarchy;
    

    findContours(imgDil, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(img, contours, i, Scalar(255, 0, 255), 2);
        
    }
    
}

int main()
{
   
    VideoCapture cap(0);
    Mat img, img2, imgHSV;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));

    for (;;)
    {
        if (cap.empty())
            break;
        cap >> img;
        cvtColor(img, imgHSV, COLOR_BGR2HSV);//色彩空间转换为HSV
        inRange(imgHSV, Scalar(35, 83, 86), Scalar(77, 200, 200), img2);//利用特定阈值将HSV后图像进行二值化处理
        getContours(img2, img);
        namedWindow("www", WINDOW_AUTOSIZE);
        imshow("www", img);

        waitKey(1);

    }
        return 0;
}
