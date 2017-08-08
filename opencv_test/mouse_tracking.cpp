// KalmanTest.cpp : Defines the entry point for the console application.
//

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Kalman Filter for mouse tracking
// OpenCV 2.1
// photo3d.apps@gmail.com
// 12/13/2013
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <tchar.h>
#include "highgui.h"
#include "cv.h"

CvPoint pt_Measurement;

void on_mouse(int event, int x, int y, int flags, void*)
{
	pt_Measurement = cvPoint(x, y);
}


int _tmain(int argc, _TCHAR* argv[])
{
	// States are position and velocity in X and Y directions; four states [X;Y;dX/dt;dY/dt]
	CvPoint pt_Prediction, pt_Correction;

	// Measurements are current position of the mouse [X;Y]
	CvMat* measurement = cvCreateMat(2, 1, CV_32FC1);

	// Matrix A (PHI) is the state transition matrix which is function of the dynamic matrix (F)
	// e.g.) dynamic matrix (F) in this case is
	// F = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0]
	// since, we have four states [X;Y;dX/dt;dY/dt] and our measurements are [X;Y]
	// State transition matrix A (or PHI) can be calculated by
	// PHI = I + F + (1/2!)*F^2 + ...
	const float A[] = { 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1 };

	// H matrix
	// Measurement matrix (z_k) has linear relationship to x_k: z_k = H_k x_k + v_k ~ N(0, R_k)
	const float H[] = { 1, 0, 0, 0,
		0, 1, 0, 0 };

	const float H_no_measurement[] = { 0, 0, 0, 0,
		0, 0, 0, 0 };

	// dynamic params (4), measurement params (2), control params (0)
	CvKalman* kalman = cvCreateKalman(4, 2, 0);

	// Copying A and H matrices to kalman
	memcpy(kalman->transition_matrix->data.fl, A, sizeof(A));
	memcpy(kalman->measurement_matrix->data.fl, H, sizeof(H));

	// Assuming that covariance matrix (Q_k) and covariance matrix for measurement (R_k) are constant matrices
	cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-6));	// Q_k	 
	cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(1e-2));	// R_k

	// P_0: initial values for covariance matrix for states (big value; will be reduced anyway)
	cvSetIdentity(kalman->error_cov_post, cvRealScalar(1e+10));

	// List for drawing traces
	std::vector <CvPoint> vt_Measurement;
	std::vector <CvPoint> vt_Prediction;
	std::vector <CvPoint> vt_Correction;

	// For writing text
	CvFont font1;
	cvInitFont(&font1, CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4, 0, 1, CV_AA);
	char str0[20] = "Kalman Prediction  ";
	char str1[20] = "Measurement (Mouse)";
	char str2[20] = "Kalman Correction  ";

	// etc.
	int k;
	int FrameCount = 0;
	int max_trace = 300;
	int measuremnt_frame = 4;	// Measurement will be available at every "measuremnt_frame" frame
	int IsMeasurementExist = 0;

	// Image to show and mouse input
	IplImage* img_src = cvCreateImage(cvSize(720, 480), IPL_DEPTH_8U, 3);
	cvNamedWindow("Kalman Filter", 1);
	cvSetMouseCallback("Kalman Filter", on_mouse, 0);

	// Loop
	for (;;)
	{
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// KALMAN FILTER
		//////////////////////////////////////////////////////////////////////////////////////////////////////////

		// Kalman Prediction (time update): (x_k)p = A x_(k-1)
		cvKalmanPredict(kalman, 0);
		pt_Prediction = cvPoint((int)kalman->state_pre->data.fl[0], (int)kalman->state_pre->data.fl[1]);

		// Checking measurement availability
		(!(FrameCount%measuremnt_frame)) ? IsMeasurementExist = 1 : IsMeasurementExist = 0;

		// Measurements (mouse position): 
		// Measurement matrix (z_k) has linear relationship to x_k: z_k = H_k x_k + v_k ~ N(0, R_k)
		measurement->data.fl[0] = (float)pt_Measurement.x;
		measurement->data.fl[1] = (float)pt_Measurement.y;

		// Copying H matrices to kalman: two cases (1. with measurement 2. without measurement)
		// without measurement case: H = 0 makes Kalman gain (K_k) zero
		// which shows same results with prediction step (which uses state transition matrix)
		if (IsMeasurementExist)
			memcpy(kalman->measurement_matrix->data.fl, H, sizeof(H));
		else
			memcpy(kalman->measurement_matrix->data.fl, H_no_measurement, sizeof(H_no_measurement));

		// Kalman Correction (measurement update): x_k = (x_k)p + K_k(z_k - H_k *(x_k)p)
		cvKalmanCorrect(kalman, measurement);
		// Retrieving corrected states
		pt_Correction = cvPoint((int)kalman->state_post->data.fl[0], (int)kalman->state_post->data.fl[1]);

		// Save point (prediction, correction, and measurement) for drawing trace
		vt_Prediction.push_back(pt_Prediction);
		vt_Correction.push_back(pt_Correction);
		if (IsMeasurementExist)
			vt_Measurement.push_back(pt_Measurement);

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// KALMAN FILTER
		//////////////////////////////////////////////////////////////////////////////////////////////////////////



		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Drawing
		//////////////////////////////////////////////////////////////////////////////////////////////////////////

		// Clear image
		cvZero(img_src);

		// Drawing traces
		for ((int)vt_Prediction.size()>max_trace ? k = (int)vt_Prediction.size() - max_trace : k = 0; k<(int)vt_Prediction.size() - 1; k++)
		{
			cvLine(img_src, vt_Prediction[k], vt_Prediction[k + 1], CV_RGB(255, 0, 0), 2, 8, 0);
			cvLine(img_src, vt_Correction[k], vt_Correction[k + 1], CV_RGB(0, 0, 255), 2, 8, 0);
		}
		for ((int)vt_Measurement.size()>max_trace / measuremnt_frame ? k = (int)vt_Measurement.size() - max_trace / measuremnt_frame :
			k = 0; k<(int)vt_Measurement.size() - 1; k++)
		{
			cvDrawCircle(img_src, vt_Measurement[k + 1], 3, CV_RGB(0, 255, 0), -1, 8, 0);
		}

		// Drawing legend
		cvPutText(img_src, str0, cvPoint(img_src->width - 180, 30), &font1, CV_RGB(255, 255, 255));
		cvPutText(img_src, str1, cvPoint(img_src->width - 180, 45), &font1, CV_RGB(255, 255, 255));
		cvPutText(img_src, str2, cvPoint(img_src->width - 180, 60), &font1, CV_RGB(255, 255, 255));
		cvLine(img_src, cvPoint(img_src->width - 16, 26), cvPoint(img_src->width - 24, 26), CV_RGB(255, 0, 0), 2, 8, 0);
		cvDrawCircle(img_src, cvPoint(img_src->width - 20, 41), 4, CV_RGB(0, 255, 0), -1, 8, 0);
		cvLine(img_src, cvPoint(img_src->width - 16, 56), cvPoint(img_src->width - 24, 56), CV_RGB(0, 0, 255), 2, 8, 0);

		// Show image
		cvShowImage("Kalman Filter", img_src);

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Drawing
		//////////////////////////////////////////////////////////////////////////////////////////////////////////


		// Key input
		if (cvWaitKey(1)>0)
			break;

		FrameCount++;
	}

	// Releases
	cvDestroyAllWindows();
	cvReleaseKalman(&kalman);
	cvReleaseImage(&img_src);

	return 0;
}