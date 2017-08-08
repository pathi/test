//	SIFT(Scale-Invariant Feature Transform)


#include "opencv.hpp"

using namespace cv;
using namespace std;

int main(void)
{
	Mat img = imread("2.jpg", IMREAD_COLOR);
	Mat A(img.rows*img.cols, 3, CV_32FC1);
	Mat B(img.rows*img.cols, 1, CV_32FC1);
	Mat result;
	
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			A.at<float>(i*img.cols + j, 0) = i;
			A.at<float>(i*img.cols + j, 1) = j;
			A.at<float>(i*img.cols + j, 2) = 1;
			B.at<float>(i*img.cols + j, 0) = img.data[i*img.cols + j];
		}
	}

	solve(A, B, result, DECOMP_SVD);
	
	cout << "result = " << result << endl;

	return 0;
}