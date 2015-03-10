// fmfrecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

bool stream = true;
bool record = false;

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
	vector<string> window_name(numCameras);

	//Initialize cameras
	for (int i = 0; i < numCameras; i++)
	{
		error = busMgr.GetCameraFromIndex(i, &guid);
		
		fcam[i].id = i;
		window_name[i] = "Camera " + to_string(i);
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

	vector<queue <Mat>> dispStream(numCameras);
	vector<queue <Image>> imageStream(numCameras);
	vector<queue <TimeStamp>> timeStamps(numCameras);

	vector<Image> img(numCameras);
	vector<TimeStamp> stamp(numCameras);
	vector<Mat> frame(numCameras);

	int key_state = 0;

	#pragma omp parallel sections num_threads(3)
	{
		#pragma omp section
		{
			vector<int> ltime(numCameras);
			vector<int> ctime(numCameras);
			vector<int> dtime(numCameras);

			int count = 0;

			while (true)
			{
				for (int i = 0; i < numCameras; i++)
				{
					img[i] = fcam[i].GrabFrame();
					stamp[i] = fcam[i].GetTimeStamp();
					frame[i] = fcam[i].convertImagetoMat(img[i]);

					ctime[i] = stamp[i].cycleCount;

					if (ctime[i] < ltime[i])
						dtime[i] = ctime[i] + (8000 - ltime[i]);
					else
						dtime[i] = ctime[i] - ltime[i];

					if (dtime[i] > 0)
						dtime[i] = 8000 / dtime[i];
					else
						dtime[i] = 0;

					ltime[i] = ctime[i];

					putText(frame[i], to_string(dtime[i]), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					if (record)
						putText(frame[i], to_string(count), Point(1000, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

				}

				count++;

				#pragma omp critical
				{
					for (int i = 0; i < numCameras; i++)
					{
						dispStream[i].push(frame[i]);
						
						if (record)
						{
							timeStamps[i].push(stamp[i]);
							imageStream[i].push(img[i]);
						}
					}
				}

				if (GetAsyncKeyState(VK_F1))
				{
					if (!key_state)
					{
						record = !record;
						count = 0;
					}

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
				bool flag = true;

				for (int i = 0; i < numCameras; i++)
					if (imageStream[i].empty())
						flag = false;

					
				if (flag)
				{
					for (int i = 0; i < numCameras; i++)
					{
						if (!fout[i].IsOpen())
						{
							fout[i].id = i;
							fout[i].Open();
							fout[i].InitHeader(imageWidth, imageHeight);
							fout[i].WriteHeader();
						}

						fout[i].WriteFrame(timeStamps[i].front(), imageStream[i].front());
						fout[i].WriteLog(timeStamps[i].front());
						fout[i].nframes++;
					}

					#pragma omp critical
					{
						for (int i = 0; i < numCameras; i++)
						{
							imageStream[i].pop();
							timeStamps[i].pop();
						}
					}
				}
				else
				{
					for (int i = 0; i < numCameras; i++)
					{
						if (!record && fout[i].IsOpen())
							fout[i].Close();
					}
				}
				
				if (!stream)
				{
					if (!flag)
					{
						if (record)
						{
							for (int i = 0; i < numCameras; i++)
								fout[i].Close();
						}

						break;
					}
				}
			}
		}

		#pragma omp section
		{
			while (true)
			{
				bool flag = true;

				for (int i = 0; i < numCameras; i++)
					if (dispStream[i].empty())
						flag = false;
					
				if (flag)
				{
					for (int i = 0; i < numCameras; i++)
						imshow(window_name[i], dispStream[i].front());

					#pragma omp critical
					{
						for (int i = 0; i < numCameras; i++)
							dispStream[i] = queue<Mat>();
					}
				}

				waitKey(1);

				if (!stream)
				{
					for (int i = 0; i < numCameras; i++)
						destroyWindow(window_name[i]);
					
					break;
				}
			}
		}
	}

	for (unsigned int i = 0; i < numCameras; i++)
		fcam[i].Stop();

	return 0;
}


