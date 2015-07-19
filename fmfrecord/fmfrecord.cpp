// fmfrecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;
using namespace moodycamel;

#define MAXFRAMES 1000

bool stream = false;

bool lrecord = false;
bool rrecord = false;

int imageWidth = 1024, imageHeight = 1024;

ReaderWriterQueue<Image> lq(100), rq(100);

ReaderWriterQueue<Image> lrec(MAXFRAMES);
ReaderWriterQueue<Image> rrec(MAXFRAMES);

ReaderWriterQueue<Mat> ldisp_frame(1), rdisp_frame(1);

int llast = 0, lfps = 0;
int rlast = 0, rfps = 0;

int ConvertTimeToFPS(int ctime, int ltime)
{
	int dtime;

	if (ctime < ltime)
		dtime = ctime + (8000 - ltime);
	else
		dtime = ctime - ltime;

	if (dtime > 0)
		dtime = 8000 / dtime;
	else
		dtime = 0;

	return dtime;
}

void OnLeftImageGrabbed(Image* pImage, const void* pCallbackData)
{
	Image img;
	
	lfps = ConvertTimeToFPS(pImage->GetTimeStamp().cycleCount, llast);
	llast = pImage->GetTimeStamp().cycleCount;

	img.DeepCopy(pImage);
	
	if (stream)
		lq.enqueue(img);
	
	return;
}

void OnRightImageGrabbed(Image* pImage, const void* pCallbackData)
{
	Image img;

	rfps = ConvertTimeToFPS(pImage->GetTimeStamp().cycleCount, rlast);
	rlast = pImage->GetTimeStamp().cycleCount;

	img.DeepCopy(pImage);
	
	if (stream)
		rq.enqueue(img);

	return;
}

int _tmain(int argc, _TCHAR* argv[])
{
	PGRcam lcam, rcam;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;

	FlyCapture2::Error error;

	error = busMgr.GetNumOfCameras(&numCameras);
	printf("Number of cameras detected: %u\n", numCameras);

	if (numCameras < 2)
	{
		printf("Insufficient number of cameras\n");
		return -1;
	}

	error = busMgr.GetCameraFromIndex(0, &guid);

	error = lcam.Connect(guid);

	error = lcam.SetCameraParameters(imageWidth, imageHeight);
	error = lcam.SetProperty(SHUTTER, 1.998);
	error = lcam.SetProperty(GAIN, 0.0);
	error = lcam.SetTrigger();
	//error = lcam.Start();
	error = lcam.cam.StartCapture(OnLeftImageGrabbed);

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}

	error = busMgr.GetCameraFromIndex(1, &guid);

	error = rcam.Connect(guid);

	error = rcam.SetCameraParameters(imageWidth, imageHeight);
	error = rcam.SetProperty(SHUTTER, 1.998);
	error = rcam.SetProperty(GAIN, 0.0);
	error = rcam.SetTrigger();
	//error = rcam.Start();
	error = rcam.cam.StartCapture(OnRightImageGrabbed);

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}

	FmfWriter lout, rout;

	int lcount = 0, rcount = 0;
	int record_key_state = 0;

	stream = true;
	
	#pragma omp parallel sections num_threads(7)
	{
		#pragma omp section
		{
			Image img;
			Mat frame;

			while (true)
			{
				if (lq.try_dequeue(img))
				{
					unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
					Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

					frame = tframe.clone();

					putText(frame, to_string(lfps), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					putText(frame, to_string(lq.size_approx()), Point((imageWidth - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					if (lrecord)
					{
						putText(frame, to_string(lcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						lrec.enqueue(img);
						lcount++;
					}

					ldisp_frame.try_enqueue(frame.clone());

					
				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			Image img;
			Mat frame;

			while (true)
			{
				if (rq.try_dequeue(img))
				{
					unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
					Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

					frame = tframe.clone();

					putText(frame, to_string(rfps), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					putText(frame, to_string(rq.size_approx()), Point((imageWidth - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					if (rrecord)
					{
						putText(frame, to_string(rcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						rrec.enqueue(img);
						rcount++;
					}

					rdisp_frame.try_enqueue(frame.clone());

				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			Image tImage;

			while (true)
			{
				if (lrec.try_dequeue(tImage))
				{
					if (!lout.IsOpen())
					{
						lout.id = 0;
						lout.Open();
						lout.InitHeader(imageWidth, imageHeight);
						lout.WriteHeader();
					}

					lout.WriteFrame(tImage);
					lout.WriteLog(tImage.GetTimeStamp());
					lout.nframes++;
				}
				else
				{
					if (!lrecord && lout.IsOpen())
						lout.Close();

					if (!stream)
						break;
				}
			}
		}

		#pragma omp section
		{
			Image tImage;

			while (true)
			{
				if (rrec.try_dequeue(tImage))
				{
					if (!rout.IsOpen())
					{
						rout.id = 1;
						rout.Open();
						rout.InitHeader(imageWidth, imageHeight);
						rout.WriteHeader();
					}

					rout.WriteFrame(tImage);
					rout.WriteLog(tImage.GetTimeStamp());
					rout.nframes++;
				}
				else
				{
					if (!rrecord && rout.IsOpen())
						rout.Close();

					if (!stream)
						break;
				}
			}
		}

		#pragma omp section
		{
			Mat frame;

			while (true)
			{

				if (ldisp_frame.try_dequeue(frame))
					imshow("Camera Left", frame);

				waitKey(1);

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			Mat frame;
			
			while (true)
			{

				if (rdisp_frame.try_dequeue(frame))
					imshow("Camera Right", frame);
			
				waitKey(1);

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			while (true)
			{
				if (GetAsyncKeyState(VK_F2))
				{
					if (!record_key_state)
					{
						lrecord = !lrecord;
						rrecord = !rrecord;

						lcount = 0;
						rcount = 0;
					}

					record_key_state = 1;
				}
				else
					record_key_state = 0;

				if (lrecord)
				{
					if (lcount == MAXFRAMES)
					{
						lcount = 0;
						lrecord = false;
					}
				}

				if (rrecord)
				{
					if (rcount == MAXFRAMES)
					{
						rcount = 0;
						rrecord = false;
					}
				}

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}
			}
		}
	}

	if (lout.IsOpen())
		lout.Close();

	if (rout.IsOpen())
		rout.Close();

	lcam.Stop();
	rcam.Stop();
	return 0;
}


