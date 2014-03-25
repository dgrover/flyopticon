// fmfrecord.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "SerialClass.h"

#include <FlyCapture2.h>
#include <omp.h>

#define TRIGGER_CAMERA	0

using namespace FlyCapture2;

FILE **fout;
FlyCapture2::Camera** ppCameras;
FlyCapture2::Error error;
char fname[10][100];

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
	int frameNumber;
	double stamp;
	FlyCapture2::Image rawImage;
	
	// Create a converted image
	FlyCapture2::Image convertedImage;

	FlyCapture2::TimeStamp timestamp;
	
	// press [ESC] to exit from continuous streaming mode
	for (frameNumber=0; frameNumber != numImages; frameNumber++) 
	{
			// TODO: arduino code to fire hardware trigger
			
			// Start capturing images
			error = ppCameras[i]->RetrieveBuffer( &rawImage );
			
			//get image timestamp
			timestamp = rawImage.GetTimeStamp();
			stamp = (double) timestamp.seconds;
						
			// Convert the raw image
			error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage );
			
			if (error != FlyCapture2::PGRERROR_OK)
			{
				PrintError( error );
			}

			fwrite(&stamp, sizeof(double), 1, fout[i]);
			fwrite(convertedImage.GetData(), convertedImage.GetDataSize(), 1, fout[i]);
			
			if (GetAsyncKeyState(VK_ESCAPE))
				break;
	}

	return frameNumber;	//return frame number in the event that [ESC] was pressed to stop camera streaming before set nframes was reached
}

int _tmain(int argc, _TCHAR* argv[])
{
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

	SYSTEMTIME st;
	GetLocalTime(&st);
		  	
    for ( unsigned int i = 0; i < numCameras; i++)
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

		//// Power on the camera
		//const unsigned int k_cameraPower = 0x610;
		//const unsigned int k_powerVal = 0x80000000;
		//error  = ppCameras[i]->WriteRegister( k_cameraPower, k_powerVal );
		//if (error != PGRERROR_OK)
		//{
		//	PrintError( error );
		//	return -1;
		//}

        // Get the camera information
        FlyCapture2::CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        PrintCameraInfo(&camInfo); 

#if TRIGGER_CAMERA
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

		//TODO: initialize arduino to fire hardware trigger

#endif

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

#if TRIGGER_CAMERA
		//Lower shutter speed for fast triggering
		FlyCapture2::Property pProp;

		pProp.type = SHUTTER;
		pProp.absControl = true;
		pProp.onePush = false;
		pProp.onOff = true;
		pProp.autoManualMode = false;
		pProp.absValue = 0.006;
#endif
		
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

    }
	
	//Starting individual cameras
	for ( unsigned int i = 0; i < numCameras; i++)
    {
		error = ppCameras[i]->StartCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			getchar();
			return -1;
		}
	}

	printf("\nGrabbing ...\n");

	//OpenMP parallel execution of camera streaming and recording to uncompressed fmf format videos
	#pragma omp parallel for num_threads(numCameras)
    for (int i = 0; i < numCameras; i++ )
    {
		nframesRun = RunSingleCamera(i, nframes);	
    }

	printf( "\nFinished grabbing %d images\n", nframesRun );
	
	for ( unsigned int i = 0; i < numCameras; i++ )
    {
		//check if number of frames streamed from the camera were the same as what was initially set
		if (nframesRun != nframes)
		{
			//seek to location in file where nframes is stored and replace
			fseek (fout[i], 20 , SEEK_SET );	
			fwrite(&nframesRun, sizeof(unsigned __int64), 1, fout[i]);
		}
		
		// close fmf flies
		fclose(fout[i]);

		// Stop capturing images       
		ppCameras[i]->StopCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}   

#if TRIGGER_CAMERA
	    // Turn off trigger mode
		triggerMode.onOff = false;
		error = ppCameras[i]->SetTriggerMode( &triggerMode );
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
