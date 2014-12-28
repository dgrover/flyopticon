// ftrack.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

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

Point3f triangulate(Mat P1, Mat P2, Point2f lpt, Point2f rpt)
{
	Mat p3D(1, 1, CV_64FC4);
	Mat lp(1, 1, CV_64FC2);
	Mat rp(1, 1, CV_64FC2);

	lp.at<double>(0, 0) = lpt.x;
	lp.at<double>(1, 0) = lpt.y;

	rp.at<double>(0, 0) = rpt.x;
	rp.at<double>(1, 0) = rpt.y;

	triangulatePoints(P1, P2, lp, rp, p3D);

	return Point3f((p3D.at<double>(0, 0) / p3D.at<double>(3, 0)), (p3D.at<double>(1, 0) / p3D.at<double>(3, 0)), (p3D.at<double>(2, 0) / p3D.at<double>(3, 0)));
}

int _tmain(int argc, _TCHAR* argv[])
{
	FileStorage fs("..\\calibration\\data\\intrinsics.xml", FileStorage::READ);
	Mat M1, M2, D1, D2;
	fs["M1"] >> M1;
	fs["M2"] >> M2;
	fs.release();
	
	fs.open("..\\calibration\\data\\extrinsics.xml", FileStorage::READ);
	Mat R, T, R1, R2, P1, P2, Q;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q;
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

			circle(lframe, lpt, 1, Scalar(255, 255, 255), -1, 8);
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

			circle(rframe, rpt, 1, Scalar(255, 255, 255), -1, 8);
		}

		pt = triangulate(P1, P2, lpt, rpt);

		printf("[%f %f %f]\n", pt.x, pt.y, pt.z);

		imshow("camera left", lframe);
		//imshow("mask left", lmask);
		
		imshow("camera right", rframe);
		//imshow("mask right", rmask);
		
		waitKey(1);
	}

	lin.Close();
	rin.Close();

	return 0;
}

