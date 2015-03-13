// ftrack.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

#define drawCross( img, center, d )\
line(img, Point(center.x - d, center.y - d), Point(center.x + d, center.y + d), Scalar(255, 255, 255));\
line(img, Point(center.x + d, center.y - d), Point(center.x - d, center.y + d), Scalar(255, 255, 255))\


bool stream = true;

float dist(Point2f p1, Point2f p2)
{
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	return(sqrt(dx*dx + dy*dy));
}

int findClosestPoint(Point2f pt, vector<Point2f> nbor)
{
	int fly_index = 0;
	if (nbor.size() == 1)
		return fly_index;
	else
	{
		float fly_dist = dist(pt, nbor[0]);

		for (int i = 1; i < nbor.size(); i++)
		{
			float res = dist(pt, nbor[i]);
			if (res < fly_dist)
			{
				fly_dist = res;
				fly_index = i;                //Store the index of nearest point
			}
		}

		return fly_index;
	}
}

//Point3f triangulate(Mat P1, Mat P2, Point2f lpt, Point2f rpt)
//{
//	Mat p3D(1, 1, CV_64FC4);
//	Mat lp(1, 1, CV_64FC2);
//	Mat rp(1, 1, CV_64FC2);
//
//	lp.at<double>(0, 0) = lpt.x;
//	lp.at<double>(1, 0) = lpt.y;
//
//	rp.at<double>(0, 0) = rpt.x;
//	rp.at<double>(1, 0) = rpt.y;
//
//	triangulatePoints(P1, P2, lp, rp, p3D);
//
//	return Point3f((float)(p3D.at<double>(0, 0) / p3D.at<double>(3, 0)), (float)(p3D.at<double>(1, 0) / p3D.at<double>(3, 0)), (float)(p3D.at<double>(2, 0) / p3D.at<double>(3, 0)));
//}

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
	
	printf("[%f %f %f %f]\n", X_homogeneous.at<double>(0, 0), X_homogeneous.at<double>(1, 0), X_homogeneous.at<double>(2, 0), X_homogeneous.at<double>(3, 0));
	
	return X_homogeneous;
}


Point2f backproject3DPoint(Mat M, Mat P, Mat pt3d)
{
	// 3D point vector [x y z 1]'
	cv::Mat point3d_vec = cv::Mat(4, 1, CV_64FC1);
	point3d_vec.at<double>(0) = pt3d.at<double>(0, 0);
	point3d_vec.at<double>(1) = pt3d.at<double>(1, 0);
	point3d_vec.at<double>(2) = pt3d.at<double>(2, 0);
	point3d_vec.at<double>(3) = pt3d.at<double>(3, 0);

	// 2D point vector [u v 1]'
	cv::Mat point2d_vec = cv::Mat(4, 1, CV_64FC1);
	point2d_vec = M * P * point3d_vec;

	// Normalization of [u v]'
	cv::Point2f point2d;
	point2d.x = point2d_vec.at<double>(0) / point2d_vec.at<double>(2);
	point2d.y = point2d_vec.at<double>(1) / point2d_vec.at<double>(2);

	return point2d;
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

	int imageWidth = 1024, imageHeight = 1024;

	FmfReader lin, rin;
	
	lin.Open(argv[1]);
	lin.ReadHeader();
	int nframes = lin.GetFrameCount();

	rin.Open(argv[2]);
	rin.ReadHeader();
	
	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

	Mat lframe, lmask, rframe, rmask;
	Ptr<BackgroundSubtractor> lpMOG2, rpMOG2;
				
	lpMOG2 = createBackgroundSubtractorMOG2();
	rpMOG2 = createBackgroundSubtractorMOG2();

	Point2f lpt, rpt;
	Point2f backlpt, backrpt;
	Point3f pt;

	for (int imageCount = 0; imageCount < nframes; imageCount++)
	{
		lframe = lin.ReadFrame(imageCount);
		rframe = rin.ReadFrame(imageCount);

		lpMOG2->apply(lframe, lmask);
		rpMOG2->apply(rframe, rmask);

		erode(lmask, lmask, erodeElement, Point(-1, -1), 1);
		dilate(lmask, lmask, dilateElement, Point(-1, -1), 1);

		erode(rmask, rmask, erodeElement, Point(-1, -1), 1);
		dilate(rmask, rmask, dilateElement, Point(-1, -1), 1);

		vector<vector<Point>> lcontours, rcontours;

		findContours(lmask, lcontours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		findContours(rmask, rcontours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		// Get the left camera moments and mass centers
		vector<Moments> lmu(lcontours.size());
		vector<Point2f> lmc(lcontours.size());

		for (int i = 0; i < lcontours.size(); i++)
		{
			//drawContours(lmask, lcontours, i, Scalar(255, 255, 255), -1, 8, vector<Vec4i>(), 0, Point());
			lmu[i] = moments(lcontours[i], false);
			lmc[i] = Point2f(lmu[i].m10 / lmu[i].m00, lmu[i].m01 / lmu[i].m00);
		}

		if (lmc.size() > 0)
		{
			int id = findClosestPoint(lpt, lmc);
			lpt = lmc[id];
		}

		// Get the right camera moments and mass centers
		vector<Moments> rmu(rcontours.size());
		vector<Point2f> rmc(rcontours.size());

		for (int i = 0; i < rcontours.size(); i++)
		{
			//drawContours(rmask, rcontours, i, Scalar(255, 255, 255), -1, 8, vector<Vec4i>(), 0, Point());
			rmu[i] = moments(rcontours[i], false);
			rmc[i] = Point2f(rmu[i].m10 / rmu[i].m00, rmu[i].m01 / rmu[i].m00);
		}

		if (rmc.size() > 0)
		{
			int id = findClosestPoint(rpt, rmc);
			rpt = rmc[id];
		}

		//pt = triangulate(M1*P1, M2*P2, lpt, rpt);
		//printf("[%f %f %f]\n", pt.x, pt.y, pt.z);

		Mat lp(1, 1, CV_64FC3);
		lp.at<double>(0, 0) = lpt.x;
		lp.at<double>(1, 0) = lpt.y;
		lp.at<double>(2, 0) = 1;

		Mat rp(1, 1, CV_64FC3);
		rp.at<double>(0, 0) = rpt.x;
		rp.at<double>(1, 0) = rpt.y;
		rp.at<double>(2, 0) = 1;

		Mat pt3d = triangulate_Linear_LS(M1*P1, M2*P2, lp, rp);

		backlpt = backproject3DPoint(M1, P1, pt3d);
		backrpt = backproject3DPoint(M2, P2, pt3d);

		//printf("[%f %f %f %f]\n", lpt.x, lpt.y, backlpt.x, backlpt.y);
		//printf("[%f %f %f %f]\n", rpt.x, rpt.y, backrpt.x, backrpt.y);

		circle(lframe, lpt, 1, Scalar(255, 255, 255), -1, 8);
		circle(rframe, rpt, 1, Scalar(255, 255, 255), -1, 8);

		drawCross(lframe, backlpt, 2);
		drawCross(rframe, backrpt, 2);

		imshow("camera left", lframe);
		//imshow("mask left", lmask);
		
		imshow("camera right", rframe);
		//imshow("mask right", rmask);
		
		waitKey(1);
	}

	lin.Close();
	rin.Close();

	getchar();

	return 0;
}

