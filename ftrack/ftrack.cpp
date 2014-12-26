// ftrack.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

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

int _tmain(int argc, _TCHAR* argv[])
{
	Mat pnts3D(1, 1, CV_64FC4);
	Mat cam0pnts(1, 1, CV_64FC2);
	Mat cam1pnts(1, 1, CV_64FC2);

	FileStorage fs("..\\calibration\\data\\extrinsics.xml", FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file\n");
		return -1;
	}

	Mat P1, P2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;

	int imageWidth = 1024, imageHeight = 1024;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;

	FlyCapture2::Error error;

	error = busMgr.GetNumOfCameras(&numCameras);
	printf("Number of cameras detected: %u\n", numCameras);

	if (numCameras < 1)
	{
		printf("Insufficient number of cameras\n");
		return -1;
	}

	PGRcam lcam, rcam;
	FmfWriter lout, rout;

	//Initialize cameras
	error = busMgr.GetCameraFromIndex(0, &guid);
	error = lcam.Connect(guid);
	error = lcam.SetCameraParameters(imageWidth, imageHeight);
	error = lcam.SetProperty(SHUTTER, 3.999);
	error = lcam.SetProperty(GAIN, 0.0);
	error = lcam.SetTrigger();
	error = lcam.Start();

	error = busMgr.GetCameraFromIndex(1, &guid);
	error = rcam.Connect(guid);
	error = rcam.SetCameraParameters(imageWidth, imageHeight);
	error = rcam.SetProperty(SHUTTER, 3.999);
	error = rcam.SetProperty(GAIN, 0.0);
	error = rcam.SetTrigger();
	error = rcam.Start();

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}

	printf("\nPress [F1] to start/stop recording. Press [ESC] to exit.\n\n");

	int key_state = 0;

	bool stream = true;
	bool record = false;

	queue <Mat> lFrameStream, rFrameStream;
	queue <Mat> lMaskStream, rMaskStream;

	queue <Image> lImageStream, rImageStream;
	queue <TimeStamp> lStampStream, rStampStream;

	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

	#pragma omp parallel sections num_threads(3)
	{
		#pragma omp section
		{
			Image limg, rimg;
			TimeStamp lstamp, rstamp;
								
			Mat lframe, lmask, rframe, rmask;
			Ptr<BackgroundSubtractor> lpMOG2, rpMOG2;
				
			lpMOG2 = createBackgroundSubtractorMOG2();
			rpMOG2 = createBackgroundSubtractorMOG2();

			Point2f lpt, rpt;

			while (true)
			{
				limg = lcam.GrabFrame();
				lstamp = lcam.GetTimeStamp();
				lframe = lcam.convertImagetoMat(limg);

				rimg = rcam.GrabFrame();
				rstamp = rcam.GetTimeStamp();
				rframe = rcam.convertImagetoMat(rimg);
					
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
				
				cam0pnts.at<double>(0, 0) = lpt.x;
				cam0pnts.at<double>(1, 0) = lpt.y;

				cam1pnts.at<double>(0, 0) = rpt.x;
				cam1pnts.at<double>(1, 0) = rpt.y;

				triangulatePoints(P1, P2, cam0pnts, cam1pnts, pnts3D);

				//printf("[%f %f]\n", cam0pnts.at<double>(0, 0), cam0pnts.at<double>(1, 0));
				//printf("[%f %f]\n", cam1pnts.at<double>(0, 0), cam1pnts.at<double>(1, 0));
				printf("[%f %f %f %f]\n", pnts3D.at<double>(0, 0) / pnts3D.at<double>(3, 0), pnts3D.at<double>(1, 0) / pnts3D.at<double>(3, 0), pnts3D.at<double>(2, 0) / pnts3D.at<double>(3, 0), pnts3D.at<double>(3, 0) / pnts3D.at<double>(3, 0));

				#pragma omp critical
				{
					lFrameStream.push(lframe);
					lMaskStream.push(lmask);

					rFrameStream.push(rframe);
					rMaskStream.push(rmask);

					lImageStream.push(limg);
					lStampStream.push(lstamp);

					rImageStream.push(rimg);
					rStampStream.push(rstamp);
				}

				if (GetAsyncKeyState(VK_F1))
				{
					if (!key_state)
						record = !record;

					key_state = 1;
				}
				else
					key_state = 0;

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}
			}
		}

		#pragma omp section
		{
			while (true)
			{
				if (!lImageStream.empty() && !lStampStream.empty() && !rImageStream.empty() && !rStampStream.empty())
				{
					if (record)
					{
						if (!lout.IsOpen())
						{
							lout.id = 0;
							lout.Open();
							lout.InitHeader(imageWidth, imageHeight);
							lout.WriteHeader();
						}

						if (!rout.IsOpen())
						{
							rout.id = 1;
							rout.Open();
							rout.InitHeader(imageWidth, imageHeight);
							rout.WriteHeader();
						}

						lout.WriteFrame(lStampStream.front(), lImageStream.front());
						lout.WriteLog(lStampStream.front());
						lout.nframes++;

						rout.WriteFrame(rStampStream.front(), rImageStream.front());
						rout.WriteLog(rStampStream.front());
						rout.nframes++;
					}
					else
					{
						if (lout.IsOpen())
							lout.Close();

						if (rout.IsOpen())
							rout.Close();
					}

					#pragma omp critical
					{
						lImageStream.pop();
						lStampStream.pop();

						rImageStream.pop();
						rStampStream.pop();
					}
				}

				printf("Recording buffer size %06d, Frames written %06d\r", lImageStream.size(), lout.nframes);

				if (lImageStream.size() == 0 && rImageStream.size() == 0 && !stream)
					break;
			}
		}

		#pragma omp section
		{
			while (true)
			{
				if (!lFrameStream.empty() && !lMaskStream.empty() && !rFrameStream.empty() && !rMaskStream.empty())
				{
					imshow("camera left", lFrameStream.front());
					//imshow("mask left", lMaskStream.front());

					imshow("camera right", rFrameStream.front());
					//imshow("mask right", rMaskStream.front());

					#pragma omp critical
					{
						lFrameStream = queue<Mat>();
						lMaskStream = queue<Mat>();

						rFrameStream = queue<Mat>();
						rMaskStream = queue<Mat>();
					}
				}

				waitKey(1);

				if (!stream)
					break;
			}
		}
	}

	destroyAllWindows();

	lcam.Stop();
	rcam.Stop();
	
	if (lout.IsOpen())
		lout.Close();

	if (rout.IsOpen())
		rout.Close();

	return 0;
}

