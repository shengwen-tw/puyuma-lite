#include <cstdio>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

#include "camera.hpp"
#include "lane_detector.hpp"
#include "self_driving.hpp"
#include "intrinsic_calibration.hpp"

using namespace cv;

enum {
	MODE_SELF_DRIVING,
	MODE_INTRINSIC_CALIB,
	MODE_EXTRINSIC_CALIB,
	MODE_COLOR_CALIB
} Puyuma_Mode;

int execute_mode;

cv::Mat camera_matrix, distort_coefficient;

void load_settings()
{
	if(!load_intrinsic_calibration("./intrinsic.yaml", camera_matrix, distort_coefficient)) {
		cout << "failed to load intrinsic parameters, please calibrate the "
			"camera first.\n";
		exit(0);
	}
}

void greeting(int argc, char **argv)
{
	if(argc == 2) {
		if(strcmp(argv[1], "run") == 0) {
			execute_mode = MODE_SELF_DRIVING;
			load_settings();
			cout << "activating self-driving system...\n";
		} else {
			goto help;
		}
	} else if(argc == 3 && (strcmp(argv[1], "-c") == 0)) {
		if(strcmp(argv[2], "intrinsic") == 0) {
			execute_mode = MODE_INTRINSIC_CALIB;
			cout << "intrinsic calibration mode.\n";
			intrinsic_calibration();
		} else if(strcmp(argv[2], "extrinsic") == 0) {
			execute_mode = MODE_EXTRINSIC_CALIB;
			cout << "extrinsic calibration mode.\n";
		} else if(strcmp(argv[2], "color") == 0) {
			execute_mode = MODE_COLOR_CALIB;
			cout << "color thresholding calibraion mode.\n";
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
		Mat map1, map2; 
		initUndistortRectifyMap(camera_matrix, distort_coefficient, Mat(), Mat(),
					Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_32F, map1, map2); 
		remap(raw_image, undistort_image, map1, map2, INTER_LINEAR); 

		cv::imshow("undistort image", undistort_image);
		waitKey(30);
#if 0
		bool get_pose = lane_estimate(undistort_image, d, phi);

                if(get_pose == true) {
                        self_driving_control(d, phi);
                } else {
                        //halt_motor();
                }
#endif
	}

	return 0;
}

__attribute__((destructor))void end()
{
	//halt_motor();
}
