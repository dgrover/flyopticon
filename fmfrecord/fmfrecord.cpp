// fmfrecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

bool stream = true;
bool record = false;

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

int _tmain(int argc, _TCHAR* argv[])
{
	//FlyWorld mov("images", "sequence.txt", "displaySettings.txt", 912, 1140, 1920);
	FlyWorld mov("images", "sequence.txt", "displaySettings.txt", 1280, 800, 1920);
	printf("%d images read [OK]\n", mov.numImages);

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
		error = fcam[i].SetProperty(SHUTTER, 1.998);
		error = fcam[i].SetProperty(GAIN, 0.0);
		error = fcam[i].SetTrigger();
		error = fcam[i].Start();

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}
	}

	//Press [F1] to start/stop recording. Press [ESC] to exit.

	vector<Mat> dispStream(numCameras);
	vector<queue <Image>> imageStream(numCameras);
	vector<queue <TimeStamp>> timeStamps(numCameras);

	vector<Image> img(numCameras);
	vector<TimeStamp> stamp(numCameras);
	vector<Mat> frame(numCameras);

	int key_state = 0;

	#pragma omp parallel sections num_threads(4)
	{
		#pragma omp section
		{
			while (true)
			{
				for (int i = 0; i < mov.numImages; i++)
				{
					for (int j = 0; j < mov.sequence[i]; j++)
					{
						mov.imageSequence->seek(((double)i) / ((double)(mov.numImages - 1)));
						mov.viewer.frame();
					}
				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			vector<int> ltime(numCameras);
			vector<int> fps(numCameras);

			int count = 0;

			while (true)
			{
				for (int i = 0; i < numCameras; i++)
				{
					img[i] = fcam[i].GrabFrame();
					stamp[i] = fcam[i].GetTimeStamp();
					frame[i] = fcam[i].convertImagetoMat(img[i]);

					fps[i] = ConvertTimeToFPS(stamp[i].cycleCount, ltime[i]);
					ltime[i] = stamp[i].cycleCount;

					putText(frame[i], to_string(fps[i]), Point((imageWidth-50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					if (record)
						putText(frame[i], to_string(count), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

				}

				count++;

				#pragma omp critical
				{
					for (int i = 0; i < numCameras; i++)
					{
						dispStream[i] = frame[i].clone();
						
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

						fout[i].WriteFrame(imageStream[i].front());
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

				#pragma omp critical
				{
					for (int i = 0; i < numCameras; i++)
						if (dispStream[i].empty())
							flag = false;

					if (flag)
					{
						for (int i = 0; i < numCameras; i++)
							imshow(window_name[i], dispStream[i]);
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


