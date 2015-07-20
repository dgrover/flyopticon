// fmfrecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;
using namespace moodycamel;

struct smat
{
	Mat frame;
	bool record;
};

struct spoint
{
	Point pt;
	bool record;
};

bool stream = false;

bool lrecord = false;
bool rrecord = false;

//bool ltrack = false;
//bool rtrack = false;

//bool computeBG = false;

int imageWidth = 1280, imageHeight = 1280;

ReaderWriterQueue<Image> lq, rq;

ReaderWriterQueue<smat> ltrk_frame, rtrk_frame;
ReaderWriterQueue<smat> ltrk_mask, rtrk_mask;
ReaderWriterQueue<spoint> ltrk_pt, rtrk_pt;

ReaderWriterQueue<Point> ldisp_pt(1), rdisp_pt(1);
ReaderWriterQueue<Mat> ldisp_mask(1), rdisp_mask(1);
ReaderWriterQueue<Mat> ldisp_frame(1), rdisp_frame(1);

int llast = 0, lfps = 0;
int rlast = 0, rfps = 0;

Mat M1, M2, D1, D2;
Mat P1, P2;

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

Mat triangulate_Linear_LS(Mat mat_P_l, Mat mat_P_r, Mat warped_back_l, Mat warped_back_r)
{
	Mat A(4, 3, CV_64FC1), b(4, 1, CV_64FC1), X(3, 1, CV_64FC1), X_homogeneous(4, 1, CV_64FC1), W(1, 1, CV_64FC1);
	W.at<double>(0, 0) = 1.0;
	A.at<double>(0, 0) = (warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 0) - mat_P_l.at<double>(0, 0);
	A.at<double>(0, 1) = (warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 1) - mat_P_l.at<double>(0, 1);
	A.at<double>(0, 2) = (warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 2) - mat_P_l.at<double>(0, 2);
	A.at<double>(1, 0) = (warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 0) - mat_P_l.at<double>(1, 0);
	A.at<double>(1, 1) = (warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 1) - mat_P_l.at<double>(1, 1);
	A.at<double>(1, 2) = (warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 2) - mat_P_l.at<double>(1, 2);
	A.at<double>(2, 0) = (warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 0) - mat_P_r.at<double>(0, 0);
	A.at<double>(2, 1) = (warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 1) - mat_P_r.at<double>(0, 1);
	A.at<double>(2, 2) = (warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 2) - mat_P_r.at<double>(0, 2);
	A.at<double>(3, 0) = (warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 0) - mat_P_r.at<double>(1, 0);
	A.at<double>(3, 1) = (warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 1) - mat_P_r.at<double>(1, 1);
	A.at<double>(3, 2) = (warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 2) - mat_P_r.at<double>(1, 2);
	b.at<double>(0, 0) = -((warped_back_l.at<double>(0, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 3) - mat_P_l.at<double>(0, 3));
	b.at<double>(1, 0) = -((warped_back_l.at<double>(1, 0) / warped_back_l.at<double>(2, 0))*mat_P_l.at<double>(2, 3) - mat_P_l.at<double>(1, 3));
	b.at<double>(2, 0) = -((warped_back_r.at<double>(0, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 3) - mat_P_r.at<double>(0, 3));
	b.at<double>(3, 0) = -((warped_back_r.at<double>(1, 0) / warped_back_r.at<double>(2, 0))*mat_P_r.at<double>(2, 3) - mat_P_r.at<double>(1, 3));

	solve(A, b, X, DECOMP_SVD);
	vconcat(X, W, X_homogeneous);

	//printf("[%f %f %f %f]\n", X_homogeneous.at<double>(0, 0), X_homogeneous.at<double>(1, 0), X_homogeneous.at<double>(2, 0), X_homogeneous.at<double>(3, 0));

	return X_homogeneous;
}

int _tmain(int argc, _TCHAR* argv[])
{
	FileStorage fs("intrinsics.xml", FileStorage::READ);
	fs["M1"] >> M1;
	fs["M2"] >> M2;
	fs["D1"] >> D1;
	fs["D2"] >> D2;
	fs.release();

	fs.open("extrinsics.xml", FileStorage::READ);
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs.release();
	
	FmfWriter fout;

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

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

	Mat lbg, rbg;

	//Mat lmean = Mat::zeros(imageWidth, imageHeight, CV_32F);
	//Mat rmean = Mat::zeros(imageWidth, imageHeight, CV_32F);

	stream = true;

	int lcount = 0;
	int rcount = 0;

	#pragma omp parallel sections num_threads(10)
	{
		#pragma omp section
		{
			Image img;
			smat in;

			bool firstframe = true;

			while (true)
			{
				if (lq.try_dequeue(img))
				{
					unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
					Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

					in.frame = tframe.clone();

					if (firstframe)
					{
						lbg = tframe.clone();
						firstframe = false;
					}

					//if (computeBG)
					//{
					//	accumulate(frame, lmean);
					//	
					//	if (lcount++ == 500)
					//	{
					//		lcount = 0;
					//		computeBG = false;
					//		lbg = lmean / 500;

					//		lbg.convertTo(lbg, CV_8UC1);
					//	}
					//}

					in.record = lrecord;
					ltrk_frame.enqueue(in);
					

					putText(tframe, to_string(lfps), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					putText(tframe, to_string(lq.size_approx()), Point((imageWidth - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					if (lrecord)
					{
						putText(tframe, to_string(lcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						lcount++;
					}

					ldisp_frame.try_enqueue(tframe.clone());
					
				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			Image img;
			smat in;

			bool firstframe = true;

			while (true)
			{
				if (rq.try_dequeue(img))
				{
					unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
					Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

					in.frame = tframe.clone();

					if (firstframe)
					{
						rbg = tframe.clone();
						firstframe = false;
					}

					//if (computeBG)
					//{
					//	accumulate(frame, rmean);

					//	if (rcount++ == 500)
					//	{
					//		rcount = 0;
					//		computeBG = false;
					//		rbg = rmean / 500;

					//		rbg.convertTo(rbg, CV_8UC1);
					//	}
					//}

					in.record = rrecord;
					rtrk_frame.enqueue(in);
										
					putText(tframe, to_string(rfps), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					putText(tframe, to_string(rq.size_approx()), Point((imageWidth - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					if (rrecord)
					{
						putText(tframe, to_string(rcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						rcount++;
					}

					rdisp_frame.try_enqueue(tframe.clone());

				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			smat frame, mask;
			Mat tmask;

			while (true)
			{
				if (ltrk_frame.try_dequeue(frame))
				{
					//if (ltrack)
					//{
						//lmog2->apply(frame, mask);

						//absdiff(frame, lbg, mask);
						subtract(lbg, frame.frame, tmask);
						threshold(tmask, tmask, 1, 255, CV_THRESH_BINARY);

						//putText(mask, to_string(ltrk_frame.size_approx()), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						//ldisp_mask.try_enqueue(mask.clone());
						
						mask.frame = tmask.clone();
						mask.record = frame.record;

						ltrk_mask.enqueue(mask);
					//}
				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			smat frame, mask;
			Mat tmask;

			while (true)
			{
				if (rtrk_frame.try_dequeue(frame))
				{
					//if (rtrack)
					//{
						//rmog2->apply(frame, mask);

						//absdiff(frame, rbg, mask);
						subtract(rbg, frame.frame, tmask);
						threshold(tmask, tmask, 1, 255, CV_THRESH_BINARY);

						//putText(mask, to_string(rtrk_frame.size_approx()), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						//rdisp_mask.try_enqueue(mask.clone());
						
						mask.frame = tmask.clone();
						mask.record = frame.record;

						rtrk_mask.enqueue(mask);
					//}
				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			smat mask;
			spoint pt;

			while (true)
			{
				Point2f lpt(-1, -1);

				if (ltrk_mask.try_dequeue(mask))
				{
					erode(mask.frame, mask.frame, element, Point(-1, -1), 3);
					dilate(mask.frame, mask.frame, element, Point(-1, -1), 3);

					vector<vector<Point>> contours;
					findContours(mask.frame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					if (contours.size() > 0)
					{
						// Get the camera moments and mass centers
						vector<Moments> mu(contours.size());
						vector<Point2f> mc(contours.size());

						double max_size = 0;
						int k;

						for (int j = 0; j < contours.size(); j++)
						{
							drawContours(mask.frame, contours, j, Scalar(255, 255, 255), CV_FILLED);

							mu[j] = moments(contours[j], false);
							mc[j] = Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00);

							double csize = contourArea(contours[j]);

							if (csize > max_size)
							{
								k = j;
								max_size = csize;
							}

							//circle(frame, mc[j], 1, Scalar(255, 255, 255), -1, 8);
						}

						lpt = mc[k];

					}

					pt.pt = lpt;
					pt.record = mask.record;

					ltrk_pt.enqueue(pt);
					ldisp_pt.try_enqueue(lpt);

					//putText(mask, to_string(ltrk_frame.size_approx()), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					ldisp_mask.try_enqueue(mask.frame.clone());
				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			smat mask;
			spoint pt;

			while (true)
			{
				Point2f rpt(-1, -1);

				if (rtrk_mask.try_dequeue(mask))
				{
					erode(mask.frame, mask.frame, element, Point(-1, -1), 3);
					dilate(mask.frame, mask.frame, element, Point(-1, -1), 3);

					vector<vector<Point>> contours;
					findContours(mask.frame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					if (contours.size() > 0)
					{
						// Get the camera moments and mass centers
						vector<Moments> mu(contours.size());
						vector<Point2f> mc(contours.size());

						double max_size = 0;
						int k;

						for (int j = 0; j < contours.size(); j++)
						{
							drawContours(mask.frame, contours, j, Scalar(255, 255, 255), CV_FILLED);

							mu[j] = moments(contours[j], false);
							mc[j] = Point2f(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00);

							double csize = contourArea(contours[j]);

							if (csize > max_size)
							{
								k = j;
								max_size = csize;
							}

							//circle(frame, mc[j], 1, Scalar(255, 255, 255), -1, 8);
						}

						rpt = mc[k];
						
					}

					pt.pt = rpt;
					pt.record = mask.record;

					rtrk_pt.enqueue(pt);
					rdisp_pt.try_enqueue(rpt);

					//putText(mask, to_string(rtrk_frame.size_approx()), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					rdisp_mask.try_enqueue(mask.frame.clone());

				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			spoint lpt, rpt;
			Mat pt3d(1, 1, CV_64FC3);

			int count = 0;

			while (true)
			{
				while (!ltrk_pt.try_dequeue(lpt)) 
				{
					if (!stream)
						break;
				}

				while (!rtrk_pt.try_dequeue(rpt)) 
				{
					if (!stream)
						break;
				}

				Mat lp(1, 1, CV_64FC3);
				lp.at<double>(0, 0) = lpt.pt.x;
				lp.at<double>(1, 0) = lpt.pt.y;
				lp.at<double>(2, 0) = 1;

				Mat rp(1, 1, CV_64FC3);
				rp.at<double>(0, 0) = rpt.pt.x;
				rp.at<double>(1, 0) = rpt.pt.y;
				rp.at<double>(2, 0) = 1;

				if ((lpt.pt.x != -1 && lpt.pt.y != -1) || (rpt.pt.x != -1 && rpt.pt.y != -1))
					pt3d = triangulate_Linear_LS(M1*P1, M2*P2, lp, rp);
				else
				{
					pt3d.at<double>(0, 0) = -1;
					pt3d.at<double>(1, 0) = -1;
					pt3d.at<double>(2, 0) = -1;
				}

				//printf("[%f %f %f]\n", pt3d.at<double>(0, 0), pt3d.at<double>(1, 0), pt3d.at<double>(2, 0));

				if (lpt.record && rpt.record)
				{
					if (!fout.IsTrajOpen())
					{
						fout.Open();
						count = 0;
					}

					fout.WriteTraj(pt3d);
					fout.nframes++;
					count++;
				}
				else
				{
					if (fout.IsTrajOpen())
					{
						fout.Close();
						count = 0;
					}
				}

				if (!stream && (count == lcount) && (count == rcount) )
					break;
			}
		}

		#pragma omp section
		{
			Mat frame, mask;
			Point pt;

			while (true)
			{

				if (ldisp_frame.try_dequeue(frame))
				{
					if (ldisp_pt.try_dequeue(pt))
						circle(frame, pt, 1, Scalar(255, 255, 255), -1, 8);

					//resize(frame, frame, Size(640, 640));
					imshow("Camera Left", frame);
				}

				if (ldisp_mask.try_dequeue(mask))
				{
					//imshow("BG Left", lbg);
					//resize(mask, mask, Size(640, 640));
					imshow("Mask Left", mask);
				}
				
				waitKey(1);

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			Mat frame, mask;
			Point pt;

			while (true)
			{

				if (rdisp_frame.try_dequeue(frame))
				{
					if (rdisp_pt.try_dequeue(pt))
						circle(frame, pt, 1, Scalar(255, 255, 255), -1, 8);

					//resize(frame, frame, Size(640, 640));
					imshow("Camera Right", frame);
				}

				if (rdisp_mask.try_dequeue(mask))
				{
					//imshow("BG Right", rbg);
					//resize(mask, mask, Size(640, 640));
					imshow("Mask Right", mask);
				}
				
				waitKey(1);

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			//int track_key_state = 0;
			int record_key_state = 0;

			while (true)
			{
				//if (GetAsyncKeyState(VK_F3))
				//	computeBG = true;
					
				//if (GetAsyncKeyState(VK_F1))
				//{
				//	if (!track_key_state)
				//	{
				//		ltrack = !ltrack;
				//		rtrack = !rtrack;
				//	}

				//	track_key_state = 1;
				//}
				//else
				//	track_key_state = 0;


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

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}
			}
		}
	}
	
	lcam.Stop();
	rcam.Stop();

	if (fout.IsTrajOpen())
		fout.Close();

	return 0;
}


