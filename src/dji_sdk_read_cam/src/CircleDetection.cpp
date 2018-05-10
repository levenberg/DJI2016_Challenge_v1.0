// CircleDetection.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
	cout << "\nThis program demonstrates circle finding with the Hough transform.\n"
		"Usage:\n"
		"./houghcircles <image_name>, Default is pic1.png\n" << endl;
}

int _tmain(int argc, _TCHAR* argv[])
{
	const char* filename = argc >= 2 ? argv[1] : "stuff.jpg";

	Mat img = imread(filename, 0);     //img�Ҷȵ�
	if (img.empty())
	{
		help();
		cout << "can not open " << filename << endl;
		return -1;
	}

	Mat cimg;
	medianBlur(img, img, 5);
	cvtColor(img, cimg, COLOR_GRAY2BGR);   //cimg��ɫ��

	vector<Vec3f> circles;
	HoughCircles(img, circles, CV_HOUGH_GRADIENT, 1, 10,
		100, 30, 1, 60 // change the last two parameters
		// (min_radius & max_radius) to detect larger circles
		);
	
	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec3i c = circles[i];
		circle(cimg, Point(c[0], c[1]), c[2], Scalar(0, 0, 255), 3, CV_AA);
		circle(cimg, Point(c[0], c[1]), 1, Scalar(0, 255, 0), 3, CV_AA);
		char words[20]; 
		sprintf_s(words, "%d", i);
		putText(cimg, words, Point(c[0], c[1]), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
	}
	Vec3i a = circles[0];
	Vec3i b = circles[1];
	Vec3i c = circles[2];
	line(cimg, Point(a[0], a[1]), Point(b[0], b[1]), Scalar(255, 0, 0), 1, CV_AA);
	line(cimg, Point(a[0], a[1]), Point(c[0], c[1]), Scalar(255, 0, 0), 1, CV_AA);
	imshow("detected circles", cimg);
	waitKey();

	return 0;
}

