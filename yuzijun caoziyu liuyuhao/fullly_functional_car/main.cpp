#include <math.h>
#include "opencv2/opencv.hpp"
#include "serialport.h"

using namespace cv;

struct targets 
{
    size_t index = 0;
    bool is_marked = false;
    Point2f center = Point(0,0);
    double area_size = 0;
    double fit_level = 0;
    double radius = 0;
};

double EllipseRadius(const cv::RotatedRect &ellipse) 
{
    return (ellipse.size.width + ellipse.size.height) * 0.5;
}

int main(){

    int mode,sentry,base;
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化
    Mapdata data;
   
    Mat element = getStructuringElement(MORPH_RECT, Size(7, 7)),element2 = getStructuringElement(MORPH_RECT, Size(3, 3));
    int edgeThresh=1,_levels=3;

    //control windows
    namedWindow("c",4);
    namedWindow("c_d",2);

    int init_lh = 50;
    int init_hh = 80;
    int init_ls = 43;
    int init_hs = 170;
    int init_lv = 170;
    int init_hv = 255;

    int init_lh_d = 53;
    int init_hh_d = 130;
    int init_ls_d = 43;
    int init_hs_d = 122;
    int init_lv_d = 150;
    int init_hv_d = 255;

    createTrackbar("lh","c",&init_lh,255);
    createTrackbar("hh","c",&init_hh,255);
    createTrackbar("ls","c",&init_ls,255);
    createTrackbar("hs","c",&init_hs,255);
    createTrackbar("lv","c",&init_lv,255);
    createTrackbar("hv","c",&init_hv,255);

    createTrackbar("lh","c_d",&init_lh_d,255);
    createTrackbar("hh","c_d",&init_hh_d,255);
    createTrackbar("ls","c_d",&init_ls_d,255);
    createTrackbar("hs","c_d",&init_hs_d,255);
    createTrackbar("lv","c_d",&init_lv_d,255);
    createTrackbar("hv","c_d",&init_hv_d,255);

    VideoCapture cap_detection(4);
    VideoCapture cap_vision(2);

    if(!cap_detection.isOpened() or !cap_vision.isOpened())
        return -1;

    //Mat windows
    namedWindow("vision",0);
    namedWindow("vision_c_cnts",1);
    namedWindow("detection_v",2);
    //namedWindow("vision_c_hsv",3);
    namedWindow("detection",1);

    //time
    timespec t1, t2;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    clock_gettime(CLOCK_MONOTONIC, &t2);

    int range_time = 200;//timeout_range

    while(1){
        
        int16_t commond = 0;
        int aim_target_index = -1;

        Scalar low_b = Scalar(getTrackbarPos("lh","c"),getTrackbarPos("ls","c"),getTrackbarPos("lv","c")),upper_b = Scalar(getTrackbarPos("hh","c"),getTrackbarPos("hs","c"),getTrackbarPos("hv","c"));
        Scalar low_b_d = Scalar(getTrackbarPos("lh","c_d"),getTrackbarPos("ls","c_d"),getTrackbarPos("lv","c_d")),upper_b_d = Scalar(getTrackbarPos("hh","c_d"),getTrackbarPos("hs","c_d"),getTrackbarPos("hv","c_d"));
        Mat src_img_v,src_frame_v,dst_hsv_frame_v,hsv_frame_v,ir_hsv_frame_v,src_frame_v_c,morphologyEx_frame_v,d2_frame_v,src_canny;
        Mat src_frame_d,hsv_frame_d,ir_hsv_frame_d,morphologyEx_frame_v_d;
        Mat kernel_big = getStructuringElement(MORPH_RECT, Size(31, 31)),kernel_smale = getStructuringElement(MORPH_RECT, Size(9, 9));

    	cap_detection >> src_frame_d;
        cap_vision >> src_img_v;

        double scale = 4;
        double scale_d = 0.5;

        int center_line = src_img_v.cols*scale / 2;
        int range_line_r = center_line + 30*scale;
        int range_line_l = center_line - 30*scale;

        Size dsize = Size(src_img_v.cols*scale, src_img_v.rows*scale);
        Size orsize = Size(src_img_v.cols, src_img_v.rows);
        
        Size dsize_d = Size(src_frame_d.cols*scale_d, src_frame_d.rows*scale_d);
        Size orsize_d = Size(src_frame_d.cols, src_frame_d.rows);

        vector<std::vector<cv::Point> > contours;
        vector<Vec4i> hierarchy;

        cvtColor(src_img_v,hsv_frame_v,COLOR_RGB2HSV,3);

        bilateralFilter(hsv_frame_v, dst_hsv_frame_v, 30, 35, 15);

        //vision
        resize(dst_hsv_frame_v,src_frame_v,dsize);
        resize(src_img_v,src_img_v,dsize);
        //detection
        resize(src_frame_d,src_frame_d,dsize_d);
        
        cvtColor(src_frame_d,hsv_frame_d,COLOR_RGB2HSV,3);
        inRange(hsv_frame_d,low_b_d,upper_b_d,ir_hsv_frame_d);
        dilate(ir_hsv_frame_d,morphologyEx_frame_v_d,kernel_big);//hard dilate

        line(src_img_v,Point(range_line_r,0),Point(range_line_r,src_img_v.rows),Scalar(255,0,255),5);
        line(src_img_v,Point(range_line_l,0),Point(range_line_l,src_img_v.rows),Scalar(255,255,0),5);
        src_frame_v_c = Mat::zeros(src_frame_v.size(),CV_8UC1);
        
        inRange(src_frame_v,low_b,upper_b,ir_hsv_frame_v);
        erode(ir_hsv_frame_v,morphologyEx_frame_v,kernel_smale);
        dilate(morphologyEx_frame_v,d2_frame_v,kernel_smale);
        Canny(d2_frame_v,src_canny,edgeThresh, edgeThresh*3, 3);
        findContours(src_canny,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);

        Mat cnt_img = Mat::zeros(src_frame_v.size(), CV_8UC3);
        vector<targets> target_circles;
        
        int max_index = 0;
        double radius_max = 0;

        for(size_t i = 0;i < contours.size();i++){

            bool if_marked = false;
            double area_s = 0;
            double radius = 0;
            targets target;

            if(contours[i].size() > 5 && contourArea(contours[i]) > 1){
                
                double fit_area_size = 0,fit_level_c = 0;
                if_marked = true;
                RotatedRect box = fitEllipse(contours[i]);
                area_s = contourArea(contours[i]);
                radius = EllipseRadius(box);
                fit_area_size = pow(radius,2);

                if(radius - radius_max > 0){
                    radius_max = radius;
                    max_index = i;
                }

                double level_pre = double(area_s/fit_area_size);
                
                fit_level_c = level_pre;
                target = {i,if_marked,box.center,area_s,fit_level_c,radius};
                
                target_circles.push_back(target);

                drawContours(cnt_img,contours,i,Scalar(128,255,255),1);
                if(target.fit_level > 0.5){
                    circle(cnt_img,box.center,radius,Scalar(0,0,255),1);
                    circle(src_img_v,box.center,radius,Scalar(0,0,255),1);
                }

            }else{

                target = {i,if_marked,Point(0,0),0,0,radius};
                //cout << target.is_marked << endl;
                target_circles.push_back(target);

            }

        }
        if(target_circles.size()>0) {

            double range = target_circles[max_index].radius * 0.015;
            vector<size_t> fetch_index;

            for(size_t i = 0;i < target_circles.size();i++){
                if(abs(target_circles[i].radius - target_circles[max_index].radius) < range){
                    fetch_index.push_back(i);
                }
            }
            
            int target_index = -1;
            int target_level = 0;

            for(size_t i = 0;i < fetch_index.size();i++){
                if(target_circles[i].fit_level > target_level && target_circles[i].is_marked == true && target_circles[i].fit_level > 0.5 && target_circles[i].area_size > 150 * scale){
                    target_level = target_circles[i].fit_level;
                    target_index = i;
                }
            }

            if(fetch_index.size()==0){
                target_index = max_index;
            }

            putText(cnt_img,"target",target_circles[target_index].center,FONT_HERSHEY_SIMPLEX,2,Scalar(0,255,0));
            putText(src_img_v,"target",target_circles[target_index].center,FONT_HERSHEY_SIMPLEX,2,Scalar(0,255,0));

            aim_target_index = target_index;
        }   
        
        if(aim_target_index == -1){

            commond = 1 ;
            //cout << "no_target" << endl;
        }else{
            cout << "Index:" << target_circles[aim_target_index].index << "|Sign:" << target_circles[aim_target_index].is_marked << "|level:" << target_circles[aim_target_index].fit_level << "|center:"<< target_circles[aim_target_index].center << "|radius:" << target_circles[aim_target_index].radius << "|area_size:" << target_circles[aim_target_index].area_size << endl;
            clock_gettime(CLOCK_MONOTONIC, &t2);
            if(target_circles[aim_target_index].center.x - range_line_l > 0 && target_circles[aim_target_index].center.x - range_line_r < 0){
                commond = 3 ;
                //cout << "progress" << endl;
            }else if(target_circles[aim_target_index].center.x - range_line_r >= 0){
                commond = 4 ;
                //cout << "right" << endl;
            }else if(target_circles[aim_target_index].center.x - range_line_l <= 0){
                commond = 5 ;
                //cout << "left" << endl;
            }
        }


        int count_target = 0;

        for(size_t di = 0;di < morphologyEx_frame_v_d.rows;di ++){
            for(size_t di2 = 0;di2 < morphologyEx_frame_v_d.cols;di2 ++){
                if(morphologyEx_frame_v_d.at<uchar>(di,di2) == 255){
                    count_target ++;
                }
            }
        }
        if(count_target - morphologyEx_frame_v_d.rows*morphologyEx_frame_v_d.cols * 0.1 > 0){
            commond = 2 ;
            //cout << "FOUND" << endl;
            clock_gettime(CLOCK_MONOTONIC, &t1);
        }

        int deltaT = (t2.tv_sec - t1.tv_sec) * 10^9;
        //cout << deltaT << "time" << endl;

        if(deltaT > range_time){
            commond = 6 ;
            //cout << "time_out"<< endl;
            clock_gettime(CLOCK_MONOTONIC, &t1);
        }
        
        for(int l = 0;l < 4;l++){
            data = {10,commond};
            port.TransformData(data);
            port.send();
            cout << commond << endl;
            // waitKey(10);
        }

        //rebulid
        resize(cnt_img,cnt_img,orsize);
        resize(src_img_v,src_img_v,orsize);
        resize(src_frame_v_c,src_frame_v_c,orsize);
        //resize(ir_hsv_frame_v,ir_hsv_frame_v,orsize);

        // resize(src_frame_d,src_frame_d,orsize_d);
        // resize(ir_hsv_frame_d,ir_hsv_frame_d,orsize_d);

        imshow("vision_c_cnts",cnt_img);
        
        src_canny.copyTo(src_frame_v_c);

        imshow("vision",src_img_v);
        imshow("detection_v",src_frame_d);
        
        //imshow("vision_c_hsv",ir_hsv_frame_v);
        imshow("detection",morphologyEx_frame_v_d);
        waitKey(1);
    }

    return 0;
}

