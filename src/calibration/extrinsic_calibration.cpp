#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>

#include "lane_detector.hpp"
#include "camera.hpp"

#define OFFSET_X 40.0f
#define OFFSET_Y 40.0f

#define SQUARE_WIDTH 40.0f
#define SQUARE_HEIGHT 40.0f

using namespace std;
using namespace cv;

void save_homography_matrix(cv::Mat& H)
{
	cv::Mat _H;
	H.copyTo(_H);

	_H.convertTo(_H, CV_64F);

	for(int i = 0; i < 3; i++) {
#if 0
		ROS_INFO("[%.5f %.5f %.5f]",
			_H.at<double>(i, 0),
			_H.at<double>(i, 1),
			_H.at<double>(i, 2)
		);
#endif
	}
}

void mark_checkboard_corners(cv::Mat& rectified_image, std::vector<cv::Point2f>& corners)
{
	cv::Mat marked_image;
	rectified_image.copyTo(marked_image);

	for(size_t i = 0; i < corners.size(); i++) {
		cv::Point2f point = corners[i];

		char index[50] = {'\0'};

		sprintf(index, "%d", i);

		cv::circle(marked_image, Point(point.x, point.y), 1, 
			   Scalar(0, 0, 255), 2, CV_AA, 0);
		putText(marked_image, index, Point(point.x, point.y + 10),
			FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0));
	}

	cv::imshow("Extrinsic calibration", marked_image);

	cvWaitKey(1);	
}

bool estimate_homography(cv::Mat& rectified_image, cv::Mat& H)
{
	std::vector<cv::Point2f> corners;

	int board_w = 7, board_h = 5;
	cv::Size board_size(board_w, board_h);

	bool found = findChessboardCorners(rectified_image, board_size,
					   corners, CV_CALIB_CB_ADAPTIVE_THRESH +
					   	    CV_CALIB_CB_FILTER_QUADS);

	if(found == true) {
		cornerSubPix(rectified_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cout << "checkerboard is found.\n";
	} else {
		cout << "cannot find the checkerboard, please adjust the light or "
			"reduce the noise.\n";

		return false;
	}

	mark_checkboard_corners(rectified_image, corners);

	//arrange corners if the order is wrong.
	cv::Point2f corner_low_right = corners[0];
	cv::Point2f corner_up_right = corners[(board_h - 1) * board_w];
	cv::Point2f corner_up_left = corners[board_h * board_w - 1];

	bool h_flipped = false, v_flipped = false;

	if(corner_up_left.x > corner_up_right.x) h_flipped = true;
	if(corner_low_right.y < corner_up_right.y) v_flipped = true;

	std::vector<cv::Point2f> ground_plane_points; //points position in 3d
	std::vector<cv::Point2f> image_plane_points;  //points position in 2d image
	ground_plane_points.resize(board_w * board_h);
	image_plane_points.resize(board_w * board_h);

	float x_offset = 0.f;
	float y_offset = -0.f;
	cv::Point2f offset= cv::Point2f(x_offset, y_offset);

	for(int row = 0; row < board_h; row++) {
		for(int column = 0; column < board_w; column++) {
			ground_plane_points[board_w * board_h - (row * board_w + column) - 1] = cv::Point2f(float(column) * SQUARE_WIDTH + OFFSET_X, float(row) * SQUARE_HEIGHT + OFFSET_Y);

			image_plane_points[row * board_w + column] =
				corners[
					(v_flipped ? board_h - 1 - row : row) * board_w +
					(h_flipped ? board_w - 1 - column : column)
				];
		}
	}

	mark_checkboard_corners(rectified_image, ground_plane_points);

	H = cv::findHomography(image_plane_points, ground_plane_points, CV_RANSAC);

	cv::Mat test;
	warpPerspective(rectified_image, test, H, rectified_image.size());

	imshow("ground projection", test);

	waitKey(0);

	cv::destroyWindow("ground projection");

	cv::destroyWindow("Extrinsic calibration");

	cout << "succeeded estimating the homography matrix.\n";

	save_homography_matrix(H);

	return true;
}

void extrinsic_calibration(void)
{
	raspicam::RaspiCam_Cv camera;
	if(camera_setup(camera, IMAGE_WIDTH, IMAGE_HEIGHT) == false) {
		cout << "failed to open the camera. - camera_setup()\n";
		exit(0);
	}

        cv::Mat raw_image, grey_image, ground_projected_image;
	cv::Mat H; //homography matrix

	bool get_H = false;

	while(1) {
		camera.grab();
		camera.retrieve(raw_image);

		cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 136.106985, 0.000000, 166.663269,
							0.000000, 136.627212, 105.393529,
							0.000000, 0.000000, 1.000000);
		cv::Mat distort_coffecient = (cv::Mat1d(1, 5) <<
			-0.246384, 0.037375, 0.000300, -0.001282, 0.000000);

		cv::Mat distort_image;
		cv::undistort(raw_image, distort_image, camera_matrix, distort_coffecient);

		//image sharpening
		Mat temp_image;
		cv::GaussianBlur(distort_image, temp_image, Size(0, 0) , 10);
		cv::addWeighted(distort_image, 1.8, temp_image, -0.8, 0, distort_image) ;

		if(get_H == false) {
			if(estimate_homography(distort_image, H) == true) {
				get_H = true;
			}
		} else {
			warpPerspective(distort_image, ground_projected_image, H,
					distort_image.size());

			cv::imshow("Homography image", ground_projected_image);
		}

		cv::imshow("Raw image", distort_image);

		waitKey(1);	
	}
}
