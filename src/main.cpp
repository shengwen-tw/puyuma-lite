#include <cstdio>
#include <opencv2/opencv.hpp>

#include "lane_detector.hpp"
#include "self_driving.hpp"

using namespace cv;

cv::Mat camera_matrix, distort_coffecient;

int main()
{
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
