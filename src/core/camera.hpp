#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

using namespace std;

bool load_intrinsic_calibration(string path, cv::Mat& camera_matrix,
				cv::Mat& distort_coefficients);
bool camera_setup(raspicam::RaspiCam_Cv& camera, int img_width, int img_height);
void camera_saver_init(string save_path, int img_width, int img_height);
void camera_save(cv::Mat &frame);

#endif
