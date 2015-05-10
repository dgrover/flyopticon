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

	//Press [F1] to start/stop recording. Press [ESC] to exit.

	vector<Mat> dispStream(numCameras);
	vector<queue <Image>> imageStream(numCameras);
	vector<queue <TimeStamp>> timeStamps(numCameras);

	vector<Image> img(numCameras);
	vector<TimeStamp> stamp(numCameras);
	vector<Mat> frame(numCameras);

	int record_key_state = 0;
	int count = 0;

	omp_set_nested(1);
	#pragma omp parallel sections num_threads(5)
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

			#pragma omp parallel for num_threads(numCameras)
			for (int i = 0; i < numCameras; i++)
			{

				while (true)
				{
						img[i] = fcam[i].GrabFrame();
						stamp[i] = fcam[i].GetTimeStamp();
						frame[i] = fcam[i].convertImagetoMat(img[i]);

						fps[i] = ConvertTimeToFPS(stamp[i].cycleCount, ltime[i]);
						ltime[i] = stamp[i].cycleCount;

						putText(frame[i], to_string(fps[i]), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

						if (record)
							putText(frame[i], to_string(count), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));


					if (i == 0)
						count++;

					#pragma omp critical
					{
							dispStream[i] = frame[i].clone();

							if (record)
							{
								timeStamps[i].push(stamp[i]);
								imageStream[i].push(img[i]);
							}
					}

					if (!stream)
						break;
				}
			}
		}

		#pragma omp section
		{
			vector<Image> tImage(numCameras);
			vector<TimeStamp> tStamp(numCameras);

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
					}

					#pragma omp critical
					{
						for (int i = 0; i < numCameras; i++)
						{
							tImage[i] = imageStream[i].front();
							tStamp[i] = timeStamps[i].front();

							imageStream[i].pop();
							timeStamps[i].pop();
						}
					}

					for (int i = 0; i < numCameras; i++)
					{
						fout[i].WriteFrame(tImage[i]);
						fout[i].WriteLog(tStamp[i]);
						fout[i].nframes++;
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
			vector<Mat> tframe(numCameras);

			#pragma omp parallel for num_threads(numCameras)
			for (int i = 0; i < numCameras; i++)
			{
				while (true)
				{
					#pragma omp critical
					{
						tframe[i] = dispStream[i].clone();
					}

					if (!tframe[i].empty())
						imshow(window_name[i], tframe[i]);

					waitKey(1);

					if (!stream)
					{
						destroyWindow(window_name[i]);

						break;
					}
				}
			}
		}

		#pragma omp section
		{
			while (true)
			{
				if (GetAsyncKeyState(VK_F1))
				{
					if (!record_key_state)
					{
						record = !record;
						count = 0;
					}

					record_key_state = 1;
				}
				else
					record_key_state = 0;

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}
			}

		}
	}

	for (unsigned int i = 0; i < numCameras; i++)
		fcam[i].Stop();

	return 0;
}


