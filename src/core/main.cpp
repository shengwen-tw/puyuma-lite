#include <cstdio>
#include <opencv2/opencv.hpp>

#include "lane_detector.hpp"
#include "self_driving.hpp"

using namespace cv;

enum {
	MODE_SELF_DRIVING,
	MODE_INTRINSIC_CALIB,
	MODE_EXTRINSIC_CALIB,
	MODE_COLOR_CALIB
} Puyuma_Mode;

int execute_mode;

cv::Mat camera_matrix, distort_coffecient;

void greeting(int argc, char **argv)
{
	if(argc == 2) {
		if(strcmp(argv[1], "run") == 0) {
			execute_mode = MODE_SELF_DRIVING;
			cout << "activating self-driving system...\n";
		} else {
			goto help;
		}
	} else if(strcmp(argv[1], "-c") == 0) {
		if(strcmp(argv[2], "intrinsic") == 0) {
			execute_mode = MODE_INTRINSIC_CALIB;
			cout << "intrinsic calibration mode.\n";
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

	float d, phi;

	lane_estimator_init();

	//self-driving system main loop
	while(1) {
		//TODO: retrieve image from pi camera
		cv::Mat frame;
		cv::Mat distort_image;

		//calibrate the fish-eye camera image before doing lane detection
		cv::undistort(frame, distort_image, camera_matrix, distort_coffecient);

		/* lane estimation */
		float d = 0, phi = 0;
		bool get_pose = lane_estimate(distort_image, d, phi);

                if(get_pose == true) {
                        self_driving_control(d, phi);
                } else {
                        //halt_motor();
                }
	}

	return 0;
}

__attribute__((destructor))void end()
{
	//halt_motor();
}
