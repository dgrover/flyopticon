// fmfrecord.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"

#define CAM_TRIGGER_CTRL 1
#define CAM_POWER_CTRL 0

#define DISPLAY 1
#define SAVE 0

using namespace std;
using namespace FlyCapture2;
using namespace cv;

FILE **fout;
FILE *flog;
FlyCapture2::Camera** ppCameras;
char fname[10][100];
char flogname[100];

char wname[10][100];

void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo )
{
    printf("%u %s %s\n", pCamInfo->serialNumber, pCamInfo->modelName, pCamInfo->sensorResolution);
}

void PrintError( FlyCapture2::Error error )
{
    error.PrintErrorTrace();
}

int RunSingleCamera(int i, int numImages)
{	
	int frameNumber = 0;
	FlyCapture2::Image rawImage;
	FlyCapture2::Error error;

	bool stream = true;

	// Create a converted image
	FlyCapture2::Image convertedImage;
	FlyCapture2::TimeStamp timestamp;

	queue <int> frameCount;
	queue <Image> rawImageStream;
	queue <Image> dispImageStream;
	queue <TimeStamp> rawTimeStamps;
	

	#pragma omp parallel sections
	{
		#pragma omp section
		{
			// press [ESC] to exit from continuous streaming mode
			while (stream)
			{
					// Start capturing images
					error = ppCameras[i]->RetrieveBuffer( &rawImage );

					//get image timestamp
					timestamp = rawImage.GetTimeStamp();

					// Convert the raw image
					error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage );
						
					if (error != FlyCapture2::PGRERROR_OK)
					{
						PrintError( error );
					}

					#pragma omp critical
					{
						dispImageStream.push(convertedImage);
						frameCount.push(frameNumber);
						rawImageStream.push(convertedImage);
						rawTimeStamps.push(timestamp);
					}
					
					frameNumber++;

					if ( GetAsyncKeyState(VK_ESCAPE) || frameNumber == numImages )
						stream = false;
			}
		}

		#pragma omp section
		{
			while (stream || !rawImageStream.empty())
			{
				if (!rawImageStream.empty())
				{
#if SAVE
					Image tImage = rawImageStream.front();
					TimeStamp tStamp = rawTimeStamps.front();

					double dtStamp = (double) tStamp.seconds;

					fwrite(&dtStamp, sizeof(double), 1, fout[i]);
					fwrite(tImage.GetData(), tImage.GetDataSize(), 1, fout[i]);

					fprintf(flog, "Cam %d - Frame %d - TimeStamp [%d %d]\n", i, frameCount.front(), tStamp.seconds, tStamp.microSeconds);
#endif
					#pragma omp critical
					{
						frameCount.pop();
						rawImageStream.pop();
						rawTimeStamps.pop();
					}
				}
			}
		}

		#pragma omp section
		{
			while (stream)
			{
				if (!dispImageStream.empty())
				{
#if DISPLAY
					Image dImage;
					dImage.DeepCopy(&dispImageStream.back());
					
					// convert to OpenCV Mat
					unsigned int rowBytes = (double)dImage.GetReceivedDataSize() / (double)dImage.GetRows();
					Mat frame = Mat(dImage.GetRows(), dImage.GetCols(), CV_8UC1, dImage.GetData(), rowBytes);		
						
					imshow(wname[i], frame);
					waitKey(1);
#endif			
					#pragma omp critical
					dispImageStream = queue<Image>();
				}
			}
		}
	}

	return frameNumber;	//return frame number in the event that [ESC] was pressed to stop camera streaming before set nframes was reached
}

int _tmain(int argc, _TCHAR* argv[])
{
	FlyCapture2::Error error;
	
	const Mode k_fmt7Mode = MODE_0;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;
	Format7Info fmt7Info;
	bool supported;
	
    FlyCapture2::PGRGuid guid;
    FlyCapture2::BusManager busMgr;
    unsigned int numCameras;

	unsigned __int32 fmfVersion;
	unsigned __int64 bytesPerChunk, nframes, nframesRun;
	unsigned __int32 sizeHeight, sizeWidth;

	TriggerMode triggerMode;

    error = busMgr.GetNumOfCameras(&numCameras);
	
    if (error != FlyCapture2::PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    printf( "Number of cameras detected: %u\n\n", numCameras );

    if ( numCameras < 1 )
    {
        printf( "Insufficient number of cameras\n" );
        getchar();
		return -1;
    }

	if (argc == 2)
		nframes = _ttoi(argv[1]);
	else
		nframes = -1;

	// initialize camera and video writer instances
	ppCameras = new FlyCapture2::Camera*[numCameras];
	fout = new FILE*[numCameras];
	flog = new FILE;

	SYSTEMTIME st;
	GetLocalTime(&st);
		  	
    for (unsigned int i = 0; i < numCameras; i++)
    {
        ppCameras[i] = new FlyCapture2::Camera();

        error = busMgr.GetCameraFromIndex( i, &guid );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        // Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

#if CAM_POWER_CTRL
		// Power on the camera
		const unsigned int k_cameraPower = 0x610;
		const unsigned int k_powerVal = 0x80000000;
		error  = ppCameras[i]->WriteRegister( k_cameraPower, k_powerVal );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		const unsigned int millisecondsToSleep = 100;
		unsigned int regVal = 0;
		unsigned int retries = 10;

		// Wait for camera to complete power-up
		do 
		{
#if defined(WIN32) || defined(WIN64)
			Sleep(millisecondsToSleep);    
#else
			usleep(millisecondsToSleep * 1000);
#endif
			error = ppCameras[i]->ReadRegister(k_cameraPower, &regVal);
			if (error == FlyCapture2::PGRERROR_TIMEOUT)
			{
				// ignore timeout errors, camera may not be responding to
				// register reads during power-up
			}
			else if (error != FlyCapture2::PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			retries--;
		} while ((regVal & k_powerVal) == 0 && retries > 0);

		// Check for timeout errors after retrying
		if (error == FlyCapture2::PGRERROR_TIMEOUT)
		{
			PrintError( error );
			return -1;
		}
#endif

        // Get the camera information
        FlyCapture2::CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        PrintCameraInfo(&camInfo); 

		// Query for available Format 7 modes
		fmt7Info.mode = k_fmt7Mode;
		error = ppCameras[i]->GetFormat7Info( &fmt7Info, &supported );
		
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
		{
			// Pixel format not supported!
			printf("Pixel format is not supported\n");
			return -1;
		}
	    
		Format7ImageSettings fmt7ImageSettings;
		fmt7ImageSettings.mode = k_fmt7Mode;
		fmt7ImageSettings.offsetX = 0;
		fmt7ImageSettings.offsetY = 0;
		fmt7ImageSettings.width = fmt7Info.maxWidth;
		fmt7ImageSettings.height = fmt7Info.maxHeight;
		fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

		bool valid;
		Format7PacketInfo fmt7PacketInfo;

		// Validate the settings to make sure that they are valid
		error = ppCameras[i]->ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo );

		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		if ( !valid )
		{
			// Settings are not valid
			printf("Format7 settings are not valid\n");
			return -1;
		}

		// Set the settings to the camera
		error = ppCameras[i]->SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket );

		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		//Lower shutter speed for fast triggering
		FlyCapture2::Property pProp;

		pProp.type = SHUTTER;
		pProp.absControl = true;
		pProp.onePush = false;
		pProp.onOff = true;
		pProp.autoManualMode = false;
		pProp.absValue = 0.006;

		error = ppCameras[i]->SetProperty( &pProp );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

		pProp.type = GAIN;
		pProp.absControl = true;
		pProp.onePush = false;
		pProp.onOff = true;
		pProp.autoManualMode = false;
		pProp.absValue = 18.062;

		error = ppCameras[i]->SetProperty( &pProp );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

#if CAM_TRIGGER_CTRL

		// Check for external trigger support
		TriggerModeInfo triggerModeInfo;
		error = ppCameras[i]->GetTriggerModeInfo( &triggerModeInfo );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		if ( triggerModeInfo.present != true )
		{
			printf( "Camera does not support external trigger! Exiting...\n" );
			return -1;
		}

	    // Get current trigger settings
		error = ppCameras[i]->GetTriggerMode( &triggerMode );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		// Set camera to trigger mode 0
		triggerMode.onOff = true;
		triggerMode.mode = 0;
		triggerMode.parameter = 0;

		// Triggering the camera externally using source 0.
		triggerMode.source = 0;

		error = ppCameras[i]->SetTriggerMode( &triggerMode );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
#endif

#if SAVE
		//settings for version 1.0 fmf header
		fmfVersion = 1;
		sizeHeight = fmt7ImageSettings.height;
		sizeWidth = fmt7ImageSettings.width;
		bytesPerChunk = sizeHeight*sizeWidth + sizeof(double);

		sprintf_s(fname[i], "E:\\Cam%d-%d%02d%02dT%02d%02d%02d.fmf", i, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
		remove(fname[i]);
		
		fout[i] = fopen(fname[i], "wb");
		
		if(fout[i]==NULL)
		{
			printf("\nError opening FMF writer. Recording terminated.");
			return -1;
		}
		
		//write FMF header data
		fwrite(&fmfVersion, sizeof(unsigned __int32), 1, fout[i]);
		fwrite(&sizeHeight, sizeof(unsigned __int32), 1, fout[i]);
		fwrite(&sizeWidth, sizeof(unsigned __int32), 1, fout[i]);
		fwrite(&bytesPerChunk, sizeof(unsigned __int64), 1, fout[i]);
		fwrite(&nframes, sizeof(unsigned __int64), 1, fout[i]);

		if (i == 0)
		{
				sprintf_s(flogname, "E:\\log-%d%02d%02dT%02d%02d%02d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
				remove(flogname);
		
				flog = fopen(flogname, "w");
		
				if(flog==NULL)
				{
					printf("\nError creating log file. Recording terminated.");
					return -1;
				}
		}
#endif

#if DISPLAY
		sprintf_s(wname[i], "camera view %d", i);
#endif

    }
	
	//Starting individual cameras
	for (unsigned int i = 0; i < numCameras; i++)
    {
		error = ppCameras[i]->StartCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			getchar();
			return -1;
		}
	}

	////Retrieve frame rate property
	//Property frmRate;
	//frmRate.type = FRAME_RATE;
	//error = ppCameras[0]->GetProperty( &frmRate );
	//if (error != PGRERROR_OK)
	//{
	//    PrintError( error );
	//    return -1;
	//}

	// printf( "\nFrame rate is %3.2f fps\n", frmRate.absValue );

	printf("\nGrabbing ...\n");

	//OpenMP parallel execution of camera streaming and recording to uncompressed fmf format videos
	omp_set_nested(1);
	#pragma omp parallel for num_threads(numCameras)
    for (int i = 0; i < numCameras; i++ )
    {
		nframesRun = (unsigned __int64)RunSingleCamera(i, nframes);	
    }

	printf( "\nFinished grabbing %d images\n", nframesRun );
	
	for (unsigned int i = 0; i < numCameras; i++ )
    {
#if SAVE
		//check if number of frames streamed from the camera were the same as what was initially set
		if (nframesRun != nframes)
		{
			//seek to location in file where nframes is stored and replace
			fseek (fout[i], 20 , SEEK_SET );	
			fwrite(&nframesRun, sizeof(unsigned __int64), 1, fout[i]);
		}
		
		// close fmf flies
		fclose(fout[i]);
#endif
		// Stop capturing images       
		ppCameras[i]->StopCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}  

#if SAVE
		fclose(flog);
#endif

#if CAM_TRIGGER_CTRL
	    // Turn off trigger mode
		triggerMode.onOff = false;
		error = ppCameras[i]->SetTriggerMode( &triggerMode );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}  
#endif

#if CAM_POWER_CTRL
		// Power off the camera
		const unsigned int k_cameraPower = 0x610;
		const unsigned int k_powerVal = 0x00000000;
		error  = ppCameras[i]->WriteRegister( k_cameraPower, k_powerVal );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
#endif

		// Disconnect the camera
		ppCameras[i]->Disconnect();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}   
        
        // Delete camera instances
		delete ppCameras[i];
    }

	// Free up memory
    delete [] ppCameras;
	
	printf("Done! Press Enter to exit...\n");
	getchar();

	return 0;
}
