#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


class Check
{
    
// public:
//     Check();
//     ~Check();

public:
    bool findball(Mat &img);

private:
    int hmin = 23;
    int hmax = 99;

    int smin = 36;
    int smax = 229;

    int vmin = 46;
    int vmax = 255;

    int element_size = 2;

    int low_gray = 86;
    int high_gray = 239;

    int low_value = 100;
    int high_value = 240;
};