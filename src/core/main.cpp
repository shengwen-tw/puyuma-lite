#include <cstdio>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

#include "camera.hpp"
#include "lane_detector.hpp"
#include "self_driving.hpp"
#include "intrinsic_calibration.hpp"
#include "extrinsic_calibration.hpp"
#include "color_calibration.hpp"

using namespace cv;

cv::Mat camera_matrix, distort_coefficient;

bool motor_halt = false;

void load_settings()
{
	if(!load_intrinsic_calibration("./intrinsic.yaml", camera_matrix, distort_coefficient)) {
		cout << "failed to load intrinsic parameters, please calibrate the "
			"camera first.\n";
		exit(0);
	}

	if(!load_extrinsic_calibration("./extrinsic.yaml")) {
		cout << "failed to load extrinsic parameters, please calibrate the "
			"camera first.\n";
		exit(0);
	}
}

void greeting(int argc, char **argv)
{
	if(argc == 2) {
		if(strcmp(argv[1], "run") == 0) {
			cout << "activating self-driving system...\n";
		} else {
			goto help;
		}
	} else if(argc == 3 && (strcmp(argv[1], "-c") == 0)) {
		if(strcmp(argv[2], "intrinsic") == 0) {
			cout << "intrinsic calibration mode.\n";
			intrinsic_calibration();
		} else if(strcmp(argv[2], "extrinsic") == 0) {
			extrinsic_calibration();
			cout << "extrinsic calibration mode.\n";
		} else if(strcmp(argv[2], "color") == 0) {
			cout << "color thresholding calibraion mode.\n";
			hsv_color_thresholding_calibration();
			motor_halt = true;
		} else {
			goto help;
		}
	} else {
		help:
		cout << "activate self-control system: ./puyuma run\n"
			"calibrate intrinsic parameters: ./puyuma -c intrinsic\n"
			"calibrate extrinsic parameters: ./puyuma -c extrinsic\n"
			"calibrate color thresholding values: ./puyuma -c color\n";
		exit(0);
	}
}

int main(int argc, char **argv)
{
	greeting(argc, argv);
	load_settings();

	raspicam::RaspiCam_Cv camera;
	if(camera_setup(camera, IMAGE_WIDTH, IMAGE_HEIGHT) == false) {
		cout << "failed to open the camera. - camera_setup()\n";
		exit(0);
	}

	lane_estimator_init();

	cv::Mat raw_image, undistort_image;
	float d = 0, phi = 0;

	//self-driving system main loop
	while(1) {
		camera.grab();
		camera.retrieve(raw_image);

		/* image undistortion and rectifying */
		cv::undistort(raw_image, undistort_image, camera_matrix, distort_coefficient);

		bool get_pose = lane_estimate(undistort_image, d, phi);

                if(get_pose == true) {
                        self_driving_control(d, phi);
                } else {
                        //halt_motor();
                }

		waitKey(1);
	}

	return 0;
}

__attribute__((destructor))void end()
{
	//halt_motor();
}
