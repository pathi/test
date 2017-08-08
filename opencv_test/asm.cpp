// Title   : Adaptive Support-Weight Approach for Correspondence Search
// Version : 1.0

#include "opencv.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <time.h>
#include <Windows.h>

using namespace std;
using namespace cv;

#define WIN_SIZE	11
#define GAMMA_C		9
#define GAMMA_P		5.5
#define T			40
#define MIN_DISP	10
#define MAX_DISP	60
#define SCALE		4

Mat bgr_to_grey(const Mat& bgr)
{
	int width = bgr.size().width;
	int height = bgr.size().height;
	Mat grey(height, width, 0);

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			uchar r = 0.333 * bgr.at<Vec3b>(y, x)[2];
			uchar g = 0.333 * bgr.at<Vec3b>(y, x)[1];
			uchar b = 0.333 * bgr.at<Vec3b>(y, x)[0];
			grey.at<uchar>(y, x) = uchar(r + g + b);
		}
	}

	return grey;
}

Mat asw(Mat left, Mat right, string type)
{
	int width = left.size().width;
	int height = left.size().height;
	int kernel_size = WIN_SIZE / 2;
	double *min_asw = (double *)calloc(width*height, sizeof(double));

	Mat depth(height, width, 0);

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
			min_asw[y*width + x] = DBL_MAX;
	}

	for (int offset = MIN_DISP; offset <= MAX_DISP; offset++)
	{
		Mat tmp(height, width, CV_8UC3);
		
		// shift image depend on type to save calculation time
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < offset; x++)
			{
				tmp.at<Vec3b>(y, x)[0] = right.at<Vec3b>(y, x)[0];
				tmp.at<Vec3b>(y, x)[1] = right.at<Vec3b>(y, x)[1];
				tmp.at<Vec3b>(y, x)[2] = right.at<Vec3b>(y, x)[2];
			}
			for (int x = offset; x < width; x++)
			{
				tmp.at<Vec3b>(y, x)[0] = right.at<Vec3b>(y, x - offset)[0];
				tmp.at<Vec3b>(y, x)[1] = right.at<Vec3b>(y, x - offset)[1];
				tmp.at<Vec3b>(y, x)[2] = right.at<Vec3b>(y, x - offset)[2];
			}
		}
		
		// calculate each pixel's ASW value
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				int start_x = max(0, x - kernel_size);
				int start_y = max(0, y - kernel_size);
				int end_x = min(width - 1, x + kernel_size);
				int end_y = min(height - 1, y + kernel_size);
				double E = 0;
				double numerator = 0;
				double denominator = 0;

				for (int i = start_y; i <= end_y; i++)
				{
					for (int j = start_x; j <= end_x; j++)
					{
						double delta_c1 = (double)(abs(left.at<Vec3b>(i, j)[0] - left.at<Vec3b>(y, x)[0])
							+ abs(left.at<Vec3b>(i, j)[1] - left.at<Vec3b>(y, x)[1])
							+ abs(left.at<Vec3b>(i, j)[2] - left.at<Vec3b>(y, x)[2]));		// 왼쪽 영상 거리차이
						double delta_c2 = (double)(abs(tmp.at<Vec3b>(i, j)[0] - tmp.at<Vec3b>(y, x)[0])
							+ abs(tmp.at<Vec3b>(i, j)[1] - tmp.at<Vec3b>(y, x)[1])
							+ abs(tmp.at<Vec3b>(i, j)[2] - tmp.at<Vec3b>(y, x)[2]));		// 오른쪽 영상 거리차이
						double delta_g = sqrt((i - y) * (i - y) + (j - x) * (j - x));
						double w1 = exp(-(delta_c1 / GAMMA_C + delta_g / GAMMA_P));
						double w2 = exp(-(delta_c2 / GAMMA_C + delta_g / GAMMA_P));
						double e = (abs(left.at<Vec3b>(i, j)[0] - tmp.at<Vec3b>(i, j)[0])
							+ abs(left.at<Vec3b>(i, j)[1] - tmp.at<Vec3b>(i, j)[1])
							+ abs(left.at<Vec3b>(i, j)[2] - tmp.at<Vec3b>(i, j)[2]));

						numerator += w1 * w2 * e;					// 분자
						denominator += w1 * w2;						// 분모
					}
				}
				E = numerator / denominator;
				
				// smaller ASW found
				if (E < min_asw[y*width + x])
				{
					min_asw[y*width + x] = E;
					// for better visualization
					depth.at<uchar>(y, x) = offset * SCALE;
				}
			}
		}
		printf("[%d]\n", offset);
	}
	
	free(min_asw);

	return depth;
}

void main()
{
	Mat left = imread("im2(cone).png");
	Mat right = imread("im6(cone).png");
	clock_t start, end;

	start = clock();
	imwrite("result.png", asw(left, right, "left"));
	end = clock();

	printf("Leading time : %.2fs\n", (double)(end - start) / (double)CLOCKS_PER_SEC);

	waitKey(0);
}