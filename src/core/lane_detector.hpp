#ifndef __LANE_DETECTOR_HPP__
#define __LANE_DETECTOR_HPP__

using namespace std;

#define IMAGE_WIDTH 320 //[pixel]
#define IMAGE_HEIGHT 240

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

/* camera geometry parameter */
#define CAMERA_TO_CENTER 10.0 //[cm]

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

enum SEGMENT_COLOR {WHITE, YELLOW, RED, UNKNOWN_COLOR};
enum {LEFT_EDGE, RIGHT_EDGE, UNKNOWN_SIDE};

typedef struct {
	int side;

	struct {
		float x1, y1;
		float x2, y2;
	} untransformed;

	float x1, y1;
	float x2, y2;

	float d, phi;
} segment_t;

void set_outer_hsv_color_thresholding(double o_h_min, double o_s_min, double o_v_min,
				      double o_h_max, double o_s_max, double o_v_max);
void set_inner_hsv_color_thresholding(double i_h_min, double i_s_min, double i_v_min,
				      double i_h_max, double i_s_max, double i_v_max);

bool load_extrinsic_calibration(string yaml_path);
void lane_estimator_init();
bool lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi);

#endif
