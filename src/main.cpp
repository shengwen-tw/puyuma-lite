#include <cstdio>
#include <opencv2/opencv.hpp>

#include "lane_detector.hpp"

using namespace cv;

int main()
{
	cv::Mat raw_image;
	float d, phi;

	lane_estimate(raw_image, d, phi);

	return 0;
}
