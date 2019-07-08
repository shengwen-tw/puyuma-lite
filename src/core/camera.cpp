#include <iostream>
#include <iomanip>
#include <raspicam/raspicam_cv.h>
#include <yaml-cpp/yaml.h>

using namespace std;

bool load_intrinsic_calibration(string path, cv::Mat& camera_matrix,
				cv::Mat& distort_coefficients)
{
	try {
		YAML::Node yaml = YAML::LoadFile(path);

		double intrinsic_array[9];
		double distort_array[5];

		for(int i = 0; i < 9; i++) {
			intrinsic_array[i] = yaml["camera_matrix"][i].as<double>();
		}

		camera_matrix = 
			(cv::Mat1d(3, 3) << 
			 intrinsic_array[0], intrinsic_array[1], intrinsic_array[2],
			 intrinsic_array[3], intrinsic_array[4], intrinsic_array[5],
			 intrinsic_array[6], intrinsic_array[7], intrinsic_array[8]);

		for(int i = 0; i < 5; i++) {
			distort_array[i] = yaml["distortion_coefficients"][i].as<double>();
		}

		distort_coefficients = 
			(cv::Mat1d(1, 5) <<
			 distort_array[0], distort_array[1],
			 distort_array[2], distort_array[3], distort_array[4]);

		cout << fixed << setprecision(5);
		cout << "camera matrix (intrinsic parameters):\n";
		for(int i = 0; i < 3; i++) {
			cout << "["  << intrinsic_array[i * 3 + 0]
		     	     << ", " << intrinsic_array[i * 3 + 1]
		     	     << ", " << intrinsic_array[i * 3 + 2] << "]\n";
		}

		cout << "distort coefficients:\n";
		cout << "["  << distort_array[0]
	     	     << ", " << distort_array[1]
	     	     << ", " << distort_array[2]
	     	     << ", " << distort_array[3]
	     	     << ", " << distort_array[4] << "]\n";
	} catch(...) {
		return false;
	}

	return true;
}

bool camera_setup(raspicam::RaspiCam_Cv& camera, int img_width, int img_height)
{
	camera.set(CV_CAP_PROP_FPS, 30);
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, img_width);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, img_height);
	camera.set(CV_CAP_PROP_BRIGHTNESS, 60);
	camera.set(CV_CAP_PROP_CONTRAST, 50);
	camera.set(CV_CAP_PROP_SATURATION, 100);
	camera.set(CV_CAP_PROP_GAIN, 1);
	camera.set(CV_CAP_PROP_EXPOSURE, 25);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 0);
	camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 10);

	return camera.open();
}
