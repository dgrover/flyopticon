#include "stdafx.h"
#include "fmfwriter.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

FmfWriter::FmfWriter()
{
	fp = NULL;
	flog = NULL;
	ftraj = NULL;

	nframes = 0;
}

int FmfWriter::Open()
{
	//fp = new FILE;
	//flog = new FILE;
	ftraj = new FILE;

	SYSTEMTIME st;
	GetLocalTime(&st);

	//sprintf_s(fname, "D:\\cam%d-%d%02d%02dT%02d%02d%02d.fmf", id, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	//remove(fname);

	//sprintf_s(flogname, "D:\\cam%d-log-%d%02d%02dT%02d%02d%02d.txt", id, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	//remove(flogname);

	sprintf_s(ftrajname, "D:\\traj-%d%02d%02dT%02d%02d%02d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	remove(flogname);


	//fopen_s(&fp, fname, "wb");

	//if(fp == NULL) // Cannot open File
	//{
	//	printf("\nError opening FMF writer. Recording terminated.");
	//	return -1;	
	//}

	//fopen_s(&flog, flogname, "w");
	//	
	//if(flog == NULL)
	//{
	//	printf("\nError creating log file. Recording terminated.");
	//	return -1;
	//}

	//return 1;

	fopen_s(&ftraj, ftrajname, "w");

	if (ftraj == NULL)
	{
		printf("\nError creating trajectory file. Recording terminated.");
		return -1;
	}

	return 1;

}

int FmfWriter::Close()
{
	//seek to location in file where nframes is stored and replace
	//fseek(fp, 20, SEEK_SET );	
	//fwrite(&nframes, sizeof(unsigned __int64), 1, fp);

	//fclose(fp);
	//fclose(flog);
	fclose(ftraj);

	//fp = NULL;
	//flog = NULL;
	ftraj = NULL;

	return 1;
}

void FmfWriter::InitHeader(unsigned __int32 x, unsigned __int32 y)
{
	//settings for version 1.0 fmf header, take image dimensions as input with number of frames set to zero
	fmfVersion = 1;
	SizeY = y;
	SizeX = x;
	bytesPerChunk = y*x + sizeof(double);
	//bytesPerChunk = y*x;
	nframes = 0;
}


void FmfWriter::WriteHeader()
{
	//write FMF header data
	fwrite(&fmfVersion, sizeof(unsigned __int32), 1, fp);
	fwrite(&SizeY, sizeof(unsigned __int32), 1, fp);
	fwrite(&SizeX, sizeof(unsigned __int32), 1, fp);
	fwrite(&bytesPerChunk, sizeof(unsigned __int64), 1, fp);
	fwrite(&nframes, sizeof(unsigned __int64), 1, fp);
}

//void FmfWriter::WriteFrame(TimeStamp st, Image img)
void FmfWriter::WriteFrame(Image img)
{
	//double dst = (double)st.seconds;
	//fwrite(&dst, sizeof(double), 1, fp);

	double dst = (double)nframes;
	fwrite(&dst, sizeof(double), 1, fp);

	fwrite(img.GetData(), img.GetDataSize(), 1, fp);
}

void FmfWriter::WriteLog(TimeStamp st)
{
	fprintf(flog, "Cam %d - Frame %d - TimeStamp [%d %d %d]\n", id, nframes, st.cycleSeconds, st.cycleCount, st.cycleOffset);
}

void FmfWriter::WriteTraj(Mat pt)
{
	fprintf(ftraj, "Frame %d - %f %f %f\n", nframes, pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));
}

int FmfWriter::IsOpen()
{
	if (fp == NULL) // Cannot open File
		return 0;
	else
		return 1;
}

int FmfWriter::IsTrajOpen()
{
	if (ftraj == NULL) // Cannot open File
		return 0;
	else
		return 1;
}

