// ftrack.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
{
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

	vector<PGRcam> fcam(numCameras);
	vector<FmfWriter> fout(numCameras);

	vector<queue <Point2f>> pts(numCameras);
	
	vector<string> window_name(numCameras);
	vector<string> mask_window_name(numCameras);

	//Initialize cameras
	for (int i = 0; i < numCameras; i++)
	{
		error = busMgr.GetCameraFromIndex(i, &guid);

		fcam[i].id = i;
		window_name[i] = "Camera " + to_string(i);
		mask_window_name[i] = "Mask " + to_string(i);

		error = fcam[i].Connect(guid);
		error = fcam[i].SetCameraParameters(imageWidth, imageHeight);
		error = fcam[i].SetProperty(SHUTTER, 3.999);
		error = fcam[i].SetProperty(GAIN, 0.0);
		error = fcam[i].SetTrigger();
		error = fcam[i].Start();

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

	}

	printf("\nPress [F1] to start/stop recording. Press [ESC] to exit.\n\n");

	omp_set_nested(1);
	#pragma omp parallel for num_threads(numCameras)
	for (int i = 0; i < numCameras; i++)
	{
		int key_state = 0;

		bool stream = true;
		bool record = false;

		queue <Mat> dispStream;
		queue <Mat> dispMask;

		queue <Image> imageStream;
		queue <TimeStamp> timeStamps;

		Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
		Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

		#pragma omp parallel sections num_threads(3)
		{
			#pragma omp section
			{
				Image img;
				TimeStamp stamp;
								
				Mat frame, mask;
				Ptr<BackgroundSubtractor> pMOG2;
				pMOG2 = createBackgroundSubtractorMOG2();

				Point2f pt;

				while (true)
				{
					img = fcam[i].GrabFrame();
					stamp = fcam[i].GetTimeStamp();
					frame = fcam[i].convertImagetoMat(img);

					pMOG2->apply(frame, mask);

					erode(mask, mask, erodeElement, Point(-1, -1), 1);
					dilate(mask, mask, dilateElement, Point(-1, -1), 1);

					vector<vector<Point>> contours;

					findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					pt = Point2f(-1, -1);

					// Get the moments and mass centers
					vector<Moments> mu(contours.size());
					vector<Point2f> mc(contours.size());

					for (int j = 0; j < contours.size(); j++)
					{
						//drawContours(mask, contours, i, Scalar(255, 255, 255), -1, 8, vector<Vec4i>(), 0, Point());
						mu[j] = moments(contours[j], false);
						mc[j] = Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00);
						circle(frame, mc[j], 1, Scalar(255, 255, 255), -1, 8);
						
						pt = mc[j];
					}

					#pragma omp critical
					{
						dispStream.push(frame);
						dispMask.push(mask);

						imageStream.push(img);
						timeStamps.push(stamp);

						pts[i].push(pt);
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
					if (!imageStream.empty())
					{
						if (record)
						{
							if (!fout[i].IsOpen())
							{
								fout[i].id = i;
								fout[i].Open();
								fout[i].InitHeader(imageWidth, imageHeight);
								fout[i].WriteHeader();
							}

							fout[i].WriteFrame(timeStamps.front(), imageStream.front());
							fout[i].WriteLog(timeStamps.front());
							fout[i].nframes++;
						}
						else
						{
							if (fout[i].IsOpen())
								fout[i].Close();
						}

						#pragma omp critical
						{
							imageStream.pop();
							timeStamps.pop();
						}
					}

					if (i==0)
						printf("Recording buffer size %06d, Frames written %06d\r", imageStream.size(), fout[i].nframes);

					if (imageStream.size() == 0 && !stream)
						break;
				}
			}

			#pragma omp section
			{
				while (true)
				{
					if (!dispStream.empty() && !dispMask.empty())
					{
						imshow(window_name[i], dispStream.front());
						//imshow(mask_window_name[i], dispMask.front());

						#pragma omp critical
						{
							dispStream = queue<Mat>();
							dispMask = queue<Mat>();
						}
					}

					waitKey(1);

					if (!stream)
						break;
				}
			}

		}
	}

	destroyAllWindows();

	for (unsigned int i = 0; i < numCameras; i++)
	{
		fcam[i].Stop();

		if (fout[i].IsOpen())
			fout[i].Close();
	}

	return 0;
}

