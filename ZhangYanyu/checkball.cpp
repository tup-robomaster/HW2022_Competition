#include "checkball.h"

bool Check::findball(Mat &img){
    Mat gray_img,blur_img,canny_img,green_img;
    Mat dst;
    Mat myimg;
    Mat finalimg;
    img.copyTo(myimg);
    cvtColor(myimg,myimg,COLOR_BGR2HSV);
    vector<cv::Mat> bgr;
    //取通道
    split(img, bgr);
    green_img = bgr.at(1);
    //找出大致绿球
    for(int row = 0; row < green_img.rows; row++){
        for(int col = 0; col < green_img.cols; col++){
            int gray = green_img.at<uchar>(row, col);
            if(gray > low_gray&&gray < high_gray)green_img.at<uchar>(row, col) = 255;
            else green_img.at<uchar>(row, col) = 0;
        }
    }
    //找绿色
    inRange(myimg,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),dst);
    //闭操作
    int s = element_size*2+1;
    Mat structureElement = getStructuringElement(MORPH_RECT,Size(s,s),Point(-1,-1));
    erode(dst,dst,structureElement,Point(-1,-1),1);
    dilate(dst,dst,structureElement,Point(-1,-1),1);
    //图像相与，找出绿球
    Mat result_img = dst&green_img;
    imshow("green ball",result_img);
    //闭操作
    Mat element2 = getStructuringElement(MORPH_ELLIPSE,Size(7,7),Point(-1,-1));
    morphologyEx(result_img,result_img,MORPH_CLOSE,element2,Point(-1,-1),1);
    //找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    vector<vector<Point>> newcontours;
    findContours(result_img,contours,hierachy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(-1,-1));
    for(size_t i = 0; i < contours.size(); i++){
        double length = arcLength(contours[i], true);
        if(length > 50)newcontours.push_back(contours[i]);
    }
    finalimg.create(result_img.size(),result_img.type());
    finalimg = Scalar(255,255,255);
    for(size_t i=0;i<newcontours.size();i++){

        Scalar color = Scalar(150,150,150);
        drawContours(finalimg,newcontours,i,color,2,8,hierachy,0,Point(0,0));
    }
    //霍夫圆检测
    vector<Vec3f> circles;
    HoughCircles(finalimg,circles,HOUGH_GRADIENT,1,30,low_value,30,2,green_img.rows/2);
    for(size_t i = 0;i<circles.size();i++){
        Vec3f cc = circles[i];
        circle(img,Point(cc[0],cc[1]),cc[2],Scalar(0,0,255),2,LINE_AA);
    }
    imshow("result",img);
    if(circles.size()>0)return 1;
    else return 0;
}