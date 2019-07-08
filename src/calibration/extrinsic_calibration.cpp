#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>

#include "lane_detector.hpp"
#include "intrinsic_calibration.hpp"
#include "camera.hpp"

#define EXT_CALIB_BOARD_W 6
#define EXT_CALIB_BOARD_H 3
#define SQUARE_WIDTH  ((float)IMAGE_WIDTH / (EXT_CALIB_BOARD_W + 1))
#define SQUARE_HEIGHT ((float)IMAGE_HEIGHT / (EXT_CALIB_BOARD_H + 1))
#define OFFSET_X SQUARE_WIDTH
#define OFFSET_Y SQUARE_HEIGHT

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

	cv::imshow("ground points visualization", marked_image);
}

bool estimate_homography(cv::Mat& rectified_image, cv::Mat& H)
{
	int board_w = EXT_CALIB_BOARD_W, board_h = EXT_CALIB_BOARD_H;
	cv::Size board_size(board_w, board_h);
	std::vector<cv::Point2f> corners;

	Mat gray_image;
	cv::cvtColor(rectified_image, gray_image, cv::COLOR_BGR2GRAY);

	bool found = findChessboardCorners(gray_image, board_size, corners);

	if(found == true) {
                TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
                cornerSubPix(gray_image, corners, Size(5, 5), Size(-1, -1), param);
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
			int gnd_point_index =
				board_w * board_h - (row * board_w + column) - 1;

			ground_plane_points[gnd_point_index] = 
				cv::Point2f(float(column) * SQUARE_WIDTH + OFFSET_X,
					    float(row) * SQUARE_HEIGHT + OFFSET_Y);

			image_plane_points[row * board_w + column] =
				corners[(v_flipped ? board_h - 1 - row : row) * board_w +
					(h_flipped ? board_w - 1 - column : column)];
		}
	}

	H = cv::findHomography(image_plane_points, ground_plane_points, CV_RANSAC);

	Mat ground_image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(0,0,0));
	mark_checkboard_corners(ground_image, ground_plane_points);

	cout << "succeeded estimating homography matrix, press ctrl+c to leave.\n";

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

        cv::Mat raw_image, ground_projected_image;
	cv::Mat H; //homography matrix
	bool got_homography = false;

	Mat camera_matrix, distort_coefficient;

	if(!load_intrinsic_calibration("./intrinsic.yaml", camera_matrix,
				       distort_coefficient)) {
		cout << "failed to load intrinsic parameters, please do "
			"intrinsic calibration first.\n";
		exit(0);
	}

	while(1) {
		camera.grab();
		camera.retrieve(raw_image);

		//image undistortion
		cv::Mat undistort_image;
		cv::undistort(raw_image, undistort_image, camera_matrix, distort_coefficient);

		//image sharpening
		Mat filtered_image;
		cv::GaussianBlur(undistort_image, filtered_image, Size(0, 0) , 10);
		cv::addWeighted(undistort_image, 1.8, filtered_image,
				-0.8, 0, undistort_image) ;

		if(got_homography == false) {
			got_homography = estimate_homography(undistort_image, H);
		} else {
			warpPerspective(undistort_image, ground_projected_image, H,
					undistort_image.size());

			cv::imshow("ground projection", ground_projected_image);
		}

		cv::imshow("raw image", undistort_image);
		waitKey(1);	
	}
}
