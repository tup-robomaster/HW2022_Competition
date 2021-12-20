#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
// #define USE_BGR_COLORSPACE
#define USE_HSV_COLORSPACE
// #define USE_CHANNEL_SUBTRACT

using namespace std;
using namespace cv;

class ball_sort
{
public:
    ball_sort(){}

    int green_ball(Mat frame);
    float calculateCircularity(vector<Point> contours_s);
    Mat bright_adjust(Mat frame);
    Mat white_balance(Mat frame);
public:
    int highH = 79;
    int lowH = 20;
    int highS = 255;
    int lowS = 20;
    int highV = 198;
    int lowV = 65;
public:
    int lowB;
    int highB;
    int lowG;
    int highG;
    int lowR; 
    int highR;
public:
    int contrast = 20; 
    int bright = 19;
    
    ~ball_sort(){}

};