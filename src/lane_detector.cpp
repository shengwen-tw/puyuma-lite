#include <cstdio>
#include <opencv2/opencv.hpp>

#include "lane_detector.hpp"

using namespace cv;

double outer_threshold_h_min, outer_threshold_s_min, outer_threshold_v_min;
double outer_threshold_h_max, outer_threshold_s_max, outer_threshold_v_max;
double inner_threshold_h_min, inner_threshold_s_min, inner_threshold_v_min;
double inner_threshold_h_max, inner_threshold_s_max, inner_threshold_v_max;

cv::Mat outer_hsv_image, inner_hsv_image;
cv::Mat outer_threshold_image, inner_threshold_image;
cv::Mat canny_image;

bool lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi)
{
	//cut region of interest image
	cv::Mat roi_image;
        cv::Rect region(0, IMAGE_HEIGHT / 2, IMAGE_WIDTH, IMAGE_HEIGHT / 2);
        roi_image = raw_image(region);

	//convert image space from rgb to hsv
	cv::cvtColor(roi_image, outer_hsv_image, COLOR_BGR2HSV);
	cv::cvtColor(roi_image, inner_hsv_image, COLOR_BGR2HSV);

	//hsv color thresholding
	cv::inRange(
		outer_hsv_image,
		Scalar(outer_threshold_h_min, outer_threshold_s_min, outer_threshold_v_min),
		Scalar(outer_threshold_h_max, outer_threshold_s_max, outer_threshold_v_max),
		outer_threshold_image
	);
	cv::inRange(
		inner_hsv_image,
		Scalar(inner_threshold_h_min, inner_threshold_s_min, inner_threshold_v_min),
		Scalar(inner_threshold_h_max, inner_threshold_s_max, inner_threshold_v_max),
		inner_threshold_image
	);

	//canny edge detection
	cv::Mat outer_gray_image, inner_gray_image;
	cv::cvtColor(roi_image, outer_gray_image, CV_BGR2GRAY);
	cv::cvtColor(roi_image, inner_gray_image, CV_BGR2GRAY);
	cv::Mat preprocess_canny_image;
	cv::Canny(outer_gray_image, preprocess_canny_image, CANNY_THRESHOLD_1,
	          CANNY_THRESHOLD_2, 3);

	//image deliation on canny image
	Mat deliation_element = cv::getStructuringElement(MORPH_RECT, Size(3, 3));
	cv::dilate(preprocess_canny_image, canny_image, deliation_element);

	//doing bitwise AND on canny image hsv thresholding image)
	cv::Mat outer_bitwise_and_image, inner_bitwise_and_image;
	cv::bitwise_and(outer_threshold_image, canny_image, outer_bitwise_and_image);
	cv::bitwise_and(inner_threshold_image, canny_image, inner_bitwise_and_image);
}
