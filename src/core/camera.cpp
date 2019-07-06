#include <raspicam/raspicam_cv.h>

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
