#include <iostream>
#include <opencv2/opencv.hpp>
#include "sort_greenball.h"
using namespace std;
using namespace cv;

int ball_sort::green_ball(Mat frame)
{
	bright_adjust(frame);
	// white_balance(frame);

#ifdef USE_CHANNEL_SUBTRACT
	vector<Mat>bgr;
  	split(frame, bgr);
 	Mat dst_image;
	subtract(bgr[1], (bgr[2] * 0.5f + bgr[0] * 0.5f), dst_image);
#endif

#ifdef USE_HSV_COLORSPACE
	Mat hsv_image, dst_image;
 	cvtColor(frame, hsv_image, COLOR_RGB2HSV);
  
  	namedWindow("dst_image", WINDOW_NORMAL);
  	createTrackbar("lowH", "dst_image", &lowH, 180);
  	createTrackbar("highH", "dst_image", &highH, 180);
  	createTrackbar("lowS", "dst_image", &lowS, 255);
 	createTrackbar("highS", "dst_image", &highS, 255);
 	createTrackbar("lowV", "dst_image", &lowV, 255);
 	createTrackbar("highV", "dst_image", &highV, 255);
 
 	inRange(hsv_image, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), dst_image);
#endif

#ifdef USE_BGR_COLORSPACE
 	Mat dst_image;
	
 	namedWindow("dst_image", WINDOW_NORMAL);
 	createTrackbar("lowB", "dst_image", &lowB, 255);
 	createTrackbar("highB", "dst_image", &highB, 255);
 	createTrackbar("lowG", "dst_image", &lowG, 255);
 	createTrackbar("highG", "dst_image", &highG, 255);
 	createTrackbar("lowR", "dst_image", &lowR, 255);
 	createTrackbar("highR", "dst_image", &highR, 255);
 
 	inRange(frame, Scalar(lowB, lowG, lowR), Scalar(highB, highG, highR), dst_image);
#endif

	threshold(dst_image, dst_image, 8, 255, THRESH_OTSU);

	Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
	morphologyEx(dst_image, dst_image, MORPH_OPEN, element);
	morphologyEx(dst_image, dst_image, MORPH_CLOSE, element);
	imshow("dst_frame", dst_image);

	vector<vector<Point>>contours;
	findContours(dst_image, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(-1,-1));
	imshow("dst_image", dst_image);

	for(int i = 1; i < contours.size(); i++)
	{
		if(contours[i].size() > 5)
		{
			RotatedRect box =  fitEllipse(contours[i]);
			float ratio = ( box.size.width / box.size.height );	//弹丸的旋转矩形长宽比接近一比一
			if( (ratio > 0.7 && ratio < 1.3)
			&&  (arcLength(contours[i], 1) > 1000 && contourArea(contours[i]) > 10000)) //面积+长度筛选
			{
				float fcircle = calculateCircularity(contours[i]);	//找出接近圆形的轮廓
				if( fcircle < 25.0f ) 
				{  
					Point2f vertex[4];
					box.points(vertex);
					Point2f Center;
					
					Center.x = (vertex[0].x + vertex[2].x) / 2;
					Center.y = (vertex[0].y + vertex[2].y) / 2;
					for(int k = 0; k < 4; k++)
					{
						line(frame, vertex[k], vertex[(k + 1) % 4], Scalar(255, 0, 0), 2);
					}

					return 1; //发现绿球，返回1给电控
				}
			}
		}
	}
	return 2; //没有发现或是白球就返回2
}

float ball_sort::calculateCircularity(vector<Point> contours_s)
{
	/**
	 * @brief 根据轮廓上的点拟合出最小包围圆得到其圆心坐标和半径，然后遍历轮廓所有点得到轮廓上的点与圆心的距离与半径的差值，并进行累加，最后求均值，均值越小越像圆
	 * 
	 * @param contours_s 
	 * @return float 
	 */
	Point2f center;
	float radius = 0;
	minEnclosingCircle(contours_s, center, radius); //找到轮廓的最小包围圆

	float fsum = 0;
	float fcompare = 0;

	//遍历轮廓上每一个点
	for (int i = 0; i < contours_s.size(); i++)
	{
		Point2f ptmp = contours_s[i];
		//计算距离
		float fdistance = sqrt( ((float)ptmp.x - center.x) * ((float)ptmp.x - center.x)
		   		       		  + ((float)ptmp.y - center.y) * ((float)ptmp.y - center.y) );
		//累加距离到圆心的差值
		float fdiff = abs(fdistance - radius);
		fsum += fdiff;
	}
	fcompare = fsum / (float)contours_s.size();
	return fcompare;
}

Mat ball_sort::bright_adjust(Mat frame)
{
    Mat dstImage = frame;
    for (int i = 0; i < frame.rows; i++)
    {
        for (int j = 0; j < frame.cols; j++)
        {
	    	//等比例增大或减小bgr三通道像素值，提高或降低亮度
            dstImage.at<Vec3b>(i, j)[0] = saturate_cast<uchar>(frame.at<Vec3b>(i, j)[0] * contrast * 0.03 + bright);
            dstImage.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(frame.at<Vec3b>(i, j)[1] * contrast * 0.03 + bright);
            dstImage.at<Vec3b>(i, j)[2] = saturate_cast<uchar>(frame.at<Vec3b>(i, j)[2] * contrast * 0.03 + bright);
        }
    }
    return dstImage;
}

Mat ball_sort::white_balance(Mat frame)
{
	Mat frame1 = frame;
	vector<Mat>rgb;
	vector<int> a(2, 0);
        
	//RGB三通道分离
	split(frame, rgb);
	a[0] = 1;
	a[1] = 2;
	//求原始图像的RGB分量的均值
	double R, G, B;
	//B = mean(rgb[0])[0];

	G = mean(rgb[1])[0];
	R = mean(rgb[2])[0];
	Scalar sca = mean(rgb[0]);
	B = sca[0];
	//需要调整的RGB分量的增益
	double KR, KG, KB;
	KB = (R + G + B) / (3 * B);
	KG = (R + G + B) / (3 * G);
	KR = (R + G + B) / (3 * R);

	//调整RGB三个通道各自的值
	rgb[0] = rgb[0] * KB;
	rgb[1] = rgb[1] * KG;
	rgb[2] = rgb[2] * KR;
 
	//RGB三通道图像合并
	merge(rgb, frame1);
	// imshow("白平衡", frame);
	return frame1;
}
