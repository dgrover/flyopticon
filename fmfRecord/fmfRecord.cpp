// fmfRecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <FlyCapture2.h>
#include <omp.h>

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
	
	//int frameNumber=0;

	// press [ESC] to exit from continuous streaming mode
	for (frameNumber=0; frameNumber != numImages; frameNumber++) 
	{
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

        // Get the camera information
        FlyCapture2::CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        PrintCameraInfo(&camInfo); 

        // Set all cameras to a specific mode and frame rate so they
        // can be synchronized
        /*error = ppCameras[i]->SetVideoModeAndFrameRate( 
            FlyCapture2::VIDEOMODE_640x480Y8, 
            FlyCapture2::FRAMERATE_60 );
        */

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

		//settings for version 1.0 fmf header
		fmfVersion = 1;
		sizeHeight = fmt7ImageSettings.height;
		sizeWidth = fmt7ImageSettings.width;
		bytesPerChunk = sizeHeight*sizeWidth + sizeof(double);

		sprintf_s(fname[i], "D:\\Camera%d.fmf", i);
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

    //Starting the capture in sync mode
    error = FlyCapture2::Camera::StartSyncCapture( numCameras, (const FlyCapture2::Camera**)ppCameras );
    
	if (error != FlyCapture2::PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
   	
	//OpenMP parallel execution of camera streaming and recording to uncompressed fmf format videos
	#pragma omp parallel for num_threads(numCameras)
    for (int i = 0; i < numCameras; i++ )
    {
		nframesRun = RunSingleCamera(i, nframes);	
    }
	
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
	
	printf("Done");
	getchar();
	return 0;
}

