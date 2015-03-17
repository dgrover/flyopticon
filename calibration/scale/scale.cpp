// scale.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

#define drawCross( img, center, d )\
line(img, Point(center.x - d, center.y - d), Point(center.x + d, center.y + d), Scalar(0, 0, 0));\
line(img, Point(center.x + d, center.y - d), Point(center.x - d, center.y + d), Scalar(0, 0, 0))\

struct {
	bool operator() (const cv::Point &pt1, const cv::Point &pt2) { return pt1.x < pt2.x; }
} mycompx;

struct {
	bool operator() (const cv::Point &pt1, const cv::Point &pt2) { return pt1.y < pt2.y; }
} mycompy;

vector<Point> detectContour(Mat frame, int thresh)
{
	Mat mask;
	threshold(frame, mask, thresh, 255, THRESH_BINARY);

	vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// Get the moments and mass centers
	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());

	double max_size = 0;
	int j;

	for (int i = 0; i < contours.size(); i++)
	{
		//drawContours(fly_frame, fly_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());

		mu[i] = moments(contours[i], false);
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

		double csize = contourArea(contours[i]);

		if (csize > max_size)
		{
			j = i;
			max_size = csize;
		}
	}

	return contours[j];
}

vector<Point> detectPerimeter(Mat frame, int min, int max)
{
	vector<Point> pts;
	vector<Point> cont = detectContour(frame, max);

	std::sort(cont.begin(), cont.end(), mycompx);

	pts.push_back(cont.front());
	pts.push_back(cont.back());

	std::sort(cont.begin(), cont.end(), mycompy);

	pts.push_back(cont.front());
	pts.push_back(cont.back());

	cont = detectContour(frame, min);

	std::sort(cont.begin(), cont.end(), mycompx);

	pts.push_back(cont.front());
	pts.push_back(cont.back());

	std::sort(cont.begin(), cont.end(), mycompy);

	pts.push_back(cont.front());
	pts.push_back(cont.back());

	return pts;
}

Mat triangulate_Linear_LS(Mat mat_P_l, Mat mat_P_r, Mat warped_back_l, Mat warped_back_r)
{
	Mat A(4, 3, CV_64FC1), b(4, 1, CV_64FC1), X(3, 1, CV_64FC1), X_homogeneous(4, 1, CV_64FC1), W(1, 1, CV_64FC1);
	W.at<double>(0, 0) = 1.0;
	A.at<double>(0, 0) = (warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 0) - mat_P_l.at<double>(0, 0);
	A.at<double>(0, 1) = (warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 1) - mat_P_l.at<double>(0, 1);
	A.at<double>(0, 2) = (warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 2) - mat_P_l.at<double>(0, 2);
	A.at<double>(1, 0) = (warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 0) - mat_P_l.at<double>(1, 0);
	A.at<double>(1, 1) = (warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 1) - mat_P_l.at<double>(1, 1);
	A.at<double>(1, 2) = (warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 2) - mat_P_l.at<double>(1, 2);
	A.at<double>(2, 0) = (warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 0) - mat_P_r.at<double>(0, 0);
	A.at<double>(2, 1) = (warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 1) - mat_P_r.at<double>(0, 1);
	A.at<double>(2, 2) = (warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 2) - mat_P_r.at<double>(0, 2);
	A.at<double>(3, 0) = (warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 0) - mat_P_r.at<double>(1, 0);
	A.at<double>(3, 1) = (warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 1) - mat_P_r.at<double>(1, 1);
	A.at<double>(3, 2) = (warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 2) - mat_P_r.at<double>(1, 2);
	b.at<double>(0, 0) = -((warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 3) - mat_P_l.at<double>(0, 3));
	b.at<double>(1, 0) = -((warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 3) - mat_P_l.at<double>(1, 3));
	b.at<double>(2, 0) = -((warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 3) - mat_P_r.at<double>(0, 3));
	b.at<double>(3, 0) = -((warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 3) - mat_P_r.at<double>(1, 3));

	solve(A, b, X, DECOMP_SVD);
	vconcat(X, W, X_homogeneous);

	//printf("[%f %f %f %f]\n", X_homogeneous.at<double>(0, 0), X_homogeneous.at<double>(1, 0), X_homogeneous.at<double>(2, 0), X_homogeneous.at<double>(3, 0));

	return X_homogeneous;
}

float dist(Point2f p1, Point2f p2)
{
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	return(sqrt(dx*dx + dy*dy));
}

int _tmain(int argc, _TCHAR* argv[])
{
	FileStorage fs("intrinsics.xml", FileStorage::READ);
	Mat M1, M2, D1, D2;
	fs["M1"] >> M1;
	fs["M2"] >> M2;
	fs["D1"] >> D1;
	fs["D2"] >> D2;
	fs.release();

	fs.open("extrinsics.xml", FileStorage::READ);
	Mat P1, P2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs.release();

	Mat lframe = imread("left.bmp", IMREAD_GRAYSCALE);
	vector<Point> lpts = detectPerimeter(lframe, 50, 150);

	Mat rframe = imread("right.bmp", IMREAD_GRAYSCALE);
	vector<Point> rpts = detectPerimeter(rframe, 45, 150);

	vector<Mat> pts3d;

	for (int i = 0; i < 8; i++)
	{
		drawCross(lframe, lpts[i], 5);
		drawCross(rframe, rpts[i], 5);

		Mat lp(1, 1, CV_64FC3);
		lp.at<double>(0, 0) = lpts[i].x;
		lp.at<double>(1, 0) = lpts[i].y;
		lp.at<double>(2, 0) = 1;

		Mat rp(1, 1, CV_64FC3);
		rp.at<double>(0, 0) = rpts[i].x;
		rp.at<double>(1, 0) = rpts[i].y;
		rp.at<double>(2, 0) = 1;

		pts3d.push_back(triangulate_Linear_LS(M1*P1, M2*P2, lp, rp));
	}

	Point2f center((pts3d[0].at<double>(0, 0) + pts3d[1].at<double>(0, 0) + pts3d[4].at<double>(0, 0) + pts3d[5].at<double>(0, 0)) / 4, (pts3d[2].at<double>(1, 0) + pts3d[3].at<double>(1, 0) + pts3d[6].at<double>(1, 0) + pts3d[7].at<double>(1, 0)) / 4);

	float zbase = 0;
	float ztop = 0;
	float radius = 0;

	for (int i = 0; i < 4; i++)
	{
		zbase += pts3d[i].at<double>(2, 0) / 4;
		ztop += pts3d[4 + i].at<double>(2, 0) / 4;

		radius += dist(center, Point2f(pts3d[i].at<double>(0, 0), pts3d[i].at<double>(1, 0))) / 4;
	}

	printf("[%f %f] %f [%f %f]\n", center.x, center.y, radius, zbase, ztop);

	imshow("left", lframe);
	imshow("right", rframe);

	waitKey();

	return 0;
}


