#ifndef __LANE_DETECTOR_HPP__
#define __LANE_DETECTOR_HPP__

using namespace std;

#define IMAGE_WIDTH 320.0 //[pixel]
#define IMAGE_HEIGHT 240.0

#define SEMI_IMAGE_WIDTH ((int)IMAGE_WIDTH / 2)
#define SEMI_IMAGE_HEIGHT ((int)IMAGE_HEIGHT/ 2)

//checkerboard parameters
#define BOARD_BOX_SIZE 3.1 //[cm]
#define BOARD_WIDTH 6.0
#define BOARD_HEIGHT 4.0

/* lane parameters */
#define L_W 5.0 //[cm[
#define L_Y 2.5
#define W 16.5

/* parameters of historgram filter */
#define DELTA_PHI 2.0 //[degree]
#define DELTA_D 2.0 //[cm]
#define PHI_MIN (-90.0)
#define PHI_MAX (+90.0)
#define D_MIN (-25.0)
#define D_MAX (+25.0)
#define HISTOGRAM_R_SIZE (int)((PHI_MAX - PHI_MIN) / DELTA_PHI) //phi
#define HISTOGRAM_C_SIZE (int)((D_MAX - D_MIN) / DELTA_D) //d
#define HISTOGRAM_FILTER_THRESHOLD (vote_count / 3)

/* lane detector parameters */
#define CANNY_THRESHOLD_1 50
#define CANNY_THRESHOLD_2 200
#define HOUGH_THRESHOLD 50
#define SIDE_DETECT_PIXEL_CNT 20
#define SIDE_DETECT_THREDHOLD 14

bool lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi);

#endif
