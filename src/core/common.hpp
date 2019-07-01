#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include <opencv2/opencv.hpp>

#define rad_to_deg(phi) (phi * 57.2957795)
#define PI 3.141592

using namespace cv;

void bound(int min, int max, int& x);
void bound(float min, float max, float& x);
float inner_product(Point2f p1, Point2f p2);
float magnatitude(Point2f p);
void normalize(Point2f& p);

#endif
