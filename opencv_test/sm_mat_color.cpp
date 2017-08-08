// Stereo Matching(Mat) Color correlation version.

#include "opencv.hpp"
#include <iostream>
#include <time.h>
#include <Windows.h>

#define MIN_DISP	10         // minimum disparity
#define MAX_DISP	60         // maximum disparity
#define W_SIZE		3         // window(patch) size, must be odd number
#define METHOD		0         // 0: SAD, 1: SSD, 2: NCC
#define SCALE		4         // disparity scale

#define COLOR_COR	1
#define ALPHA		0.95

using namespace cv;

int col, row;

double SumOfAbsoluteDifferences(Mat left, Mat right, IplImage *left_gray, IplImage *right_gray, int i, int j, int d, int p1, int p2, int q1, int q2)
{
	double diff, diff1, diff2, diff3;
	int count = 0;
	double sum = 0.0;

	for (int p = p1; p <= p2; p++)
	{
		for (int q = q1; q <= q2; q++)
		{
			double t1, t2;

			if (j - d + q<0 || j - d + q > col - 1)
				continue;
			else
			{
#if COLOR_COR
				diff	= left.at<Vec3b>(i + p, j + q)[0] - right.at<Vec3b>(i + p, j - d + q)[0];
				diff1	= left.at<Vec3b>(i + p, j + q)[1] - right.at<Vec3b>(i + p, j - d + q)[1];
				diff2	= left.at<Vec3b>(i + p, j + q)[2] - right.at<Vec3b>(i + p, j - d + q)[2];
				diff3	= left_gray->imageData[(i+p)*left_gray->widthStep + (j+q)] - right_gray->imageData[(i+p)*left_gray->widthStep + (j-d+q)];
#else
				diff = left.at<uchar>(i + p, j + q) - right.at<uchar>(i + p, j - d + q);
#endif
				count++;
			}

			if (diff < 0)			diff *= (-1);
			if (diff1 < 0)			diff1 *= (-1);
			if (diff2 < 0)			diff2 *= (-1);
			if (diff3 < 0)			diff3 *= (-1);

			t1 = (diff + diff1 + diff2) / 3.0;
			t2 = diff3;

			sum += t1*ALPHA + (1-ALPHA)*t2;
		}
	}

	if (sum != 0)
		sum /= (double)count;

	return sum;
}
double SumOfSquaredDifferences(uchar *left, uchar *right, int i, int j, int d, int p1, int p2, int q1, int q2)
{
	/*
	float diff;
	int count = 0;
	double sum = 0.0;

	for (int p = p1; p <= p2; p++)
	{
	for (int q = q1; q <= q2; q++)
	{
	if (j - d + q<0 || j - d + q > width - 1)
	continue;
	else
	{
	diff = (float)(left->imageData[(i + p)*widthStep + (j + q)] - right->imageData[(i + p)*widthStep + (j - d + q)]);
	count++;
	}

	sum += diff*diff;
	}
	}

	if (sum != 0)
	sum /= (float)count;

	return sum;
	*/
	return 0;
}


double NormalizedCrossCorrelation(uchar *left, uchar *right, int i, int j, int d, int p1, int p2, int q1, int q2)
{
	/*
	float meanL, meanR, stdL, stdR;
	int count = 0;
	double sumL = 0.0, sumR = 0.0, sum = 0;

	meanL = 0;
	meanR = 0;

	for (int p = p1; p <= p2; p++)
	{
	for (int q = q1; q <= q2; q++)
	{
	if (j - d + q<0 || j - d + q>width - 1)
	continue;
	else
	{
	sumL += left->imageData[(i+p)*widthStep + (j+q)];
	sumR += right->imageData[(i+p)*widthStep + (j-d+q)];

	count++;
	}
	}
	}

	if (count != 0)
	{
	meanL = (float)(sumL / (double)count);
	meanR = (float)(sumR / (double)count);
	}

	count = 0;
	sumL = 0.0;
	sumR = 0.0;
	stdL = 0;
	stdR = 0;

	for (int p = p1; p <= p2; p++)
	{
	for (int q = q1; q <= q2; q++)
	{
	if (j - d + q<0 || j - d + q>width - 1)
	continue;
	else
	{
	sumL += pow(left->imageData[(i + p)*widthStep + (j + q)] - meanL, 2);
	sumR += pow(right->imageData[(i + p)*widthStep + (j - d + q)] - meanR, 2);

	count++;
	}
	}
	}

	if (count != 0)
	{
	stdL = (float)(sumL / (double)count);
	stdR = (float)(sumR / (double)count);
	}

	count = 0;

	for (int p = p1; p <= p2; p++)
	{
	for (int q = q1; q <= q2; q++)
	{
	if (j - d + q<0 || j - d + q>width - 1)
	continue;
	else
	{
	sum += (left->imageData[(i + p)*widthStep + (j + q)] - meanL)*(right->imageData[(i + p)*widthStep + (j - d + q)] - meanR) / sqrt(stdL*stdR);
	count++;
	}
	}
	}

	if (count != 0)
	sum /= (double)count;

	return sum;
	*/
	return 0;
}
Mat StereoMatching(Mat left, Mat right)
{
	Mat disparity(row, col, CV_8U, Scalar(0));
	IplImage *left_gray = cvLoadImage("im2(cone).png", CV_LOAD_IMAGE_GRAYSCALE);
	IplImage *right_gray = cvLoadImage("im6(cone).png", CV_LOAD_IMAGE_GRAYSCALE);
	int w_search = W_SIZE / 2;
	double cost, min_cost, max_cost;
	int disp = 0;

	// variables for patch guard
	int guard1_i = 0, guard2_i = 0;
	int guard1_j = 0, guard2_j = 0;

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			min_cost = INT_MAX;
			max_cost = INT_MIN;

			for (int d = MIN_DISP; d < MAX_DISP; d++)
			{
				if (j - d < 0)
					continue;

				guard1_i = 0; guard2_i = 0;
				guard1_j = 0; guard2_j = 0;

				if (i == 0)
					guard1_i = w_search;
				else if (i>0 && i < w_search)
					guard1_i = w_search - i;
				else if (i == row - 1)
					guard2_i = -w_search;
				else if (i<row - 1 && i>row - 1 - w_search)
					guard2_i = -w_search + (row - 1 - i);
				else
					guard1_i = guard2_i = 0;

				if (j == 0)
					guard1_j = w_search;
				else if (j>0 && j < w_search)
					guard1_j = w_search - j;
				else if (j == col - 1)
					guard2_j = -w_search;
				else if (j<col - 1 && j>col - 1 - w_search)
					guard2_j = -w_search + (col - 1 - j);
				else
					guard1_j = guard2_j = 0;

				if (METHOD == 0)
				{
					cost = SumOfAbsoluteDifferences(left, right, left_gray, right_gray, i, j, d, -w_search + guard1_i, w_search + guard2_i, -w_search + guard1_j, w_search + guard2_j);
				}
				else if (METHOD == 1)
				{
					//cost = SumOfSquaredDifferences(left, right, i, j, d, -w_search + guard1_i, w_search + guard2_i, -w_search + guard1_j, w_search + guard2_j);
				}
				else
				{
					//cost = NormalizedCrossCorrelation(left, right, i, j, d, -w_search + guard1_i, w_search + guard2_i, -w_search + guard1_j, w_search + guard2_j);
				}

				if (METHOD < 2)
				{
					if (cost < min_cost)
					{
						min_cost = cost;
						disp = d;
					}
				}
				else
				{
					if (cost>max_cost)
					{
						max_cost = cost;
						disp = d;
					}
				}

				disparity.at<uchar>(i, j) = (uchar)disp * SCALE;
			}
		}
	}

	cvReleaseImage(&left_gray);
	cvReleaseImage(&right_gray);

	return disparity;
}
void main()
{
	if (W_SIZE % 2 == 0 || W_SIZE < 2)
	{
		printf("Window size must be odd number or greater than 3...\n");
		return;
	}

	if (METHOD > 2 || METHOD < 0)
	{
		printf("Check the matching method... 0: SAD, 1: SSD, and 2: NCC\n");
		return;
	}

	if (MIN_DISP<0 || MAX_DISP>255 || MIN_DISP >= MAX_DISP)
	{
		printf("Check the minimum and maximum disparity values...\n");
		return;
	}

	if (MAX_DISP * SCALE > 255)
	{
		printf("Scale is too large...\n");
		return;
	}

#if COLOR_COR
	Mat left_img = imread("im2(cone).png", IMREAD_COLOR);
	Mat right_img = imread("im6(cone).png", IMREAD_COLOR);
#else
	Mat left_img = imread("im2.png", IMREAD_GRAYSCALE);
	Mat right_img = imread("im6.png", IMREAD_GRAYSCALE);
#endif

	row = left_img.rows;
	col = left_img.cols;

	Mat left_disp(row, col, CV_8U, Scalar(0));
	clock_t start, end;

	start = clock();

	/* ------------------------------------------------------ */

	left_disp = StereoMatching(left_img, right_img);

	/* ------------------------------------------------------ */

	end = clock();

	printf("시간 측정 : %.4f초\n", (double)(end - start) / (double)CLOCKS_PER_SEC);

	imshow("result.pgm", left_disp); waitKey(0);
	imwrite("result.pgm", left_disp);
}