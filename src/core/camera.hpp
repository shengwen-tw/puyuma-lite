#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <raspicam/raspicam_cv.h>

bool camera_setup(raspicam::RaspiCam_Cv& camera, int img_width, int img_height);

#endif
