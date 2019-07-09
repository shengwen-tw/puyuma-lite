#include <cstdio>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "lane_detector.hpp"
#include "common.hpp"

using namespace std;
using namespace cv;

double outer_threshold_h_min, outer_threshold_s_min, outer_threshold_v_min;
double outer_threshold_h_max, outer_threshold_s_max, outer_threshold_v_max;
double inner_threshold_h_min, inner_threshold_s_min, inner_threshold_v_min;
double inner_threshold_h_max, inner_threshold_s_max, inner_threshold_v_max;

float roi_offset_x, roi_offset_y;

bool show_hsv_threshold_image = true;

cv::Mat H;

cv::Mat outer_hsv_image, inner_hsv_image;
cv::Mat outer_threshold_image, inner_threshold_image;
cv::Mat canny_image;

void set_outer_hsv_color_thresholding(double o_h_min, double o_s_min, double o_v_min,
				      double o_h_max, double o_s_max, double o_v_max)
{
	outer_threshold_h_min = o_h_min, outer_threshold_s_min = o_s_min, outer_threshold_v_min = o_v_min;
	outer_threshold_h_max = o_h_max, outer_threshold_s_max = o_h_max, outer_threshold_v_max = o_v_max;
}

void set_inner_hsv_color_thresholding(double i_h_min, double i_s_min, double i_v_min,
				      double i_h_max, double i_s_max, double i_v_max)
{
	inner_threshold_h_min = i_h_min, inner_threshold_s_min = i_s_min, inner_threshold_v_min = i_v_min;
	inner_threshold_h_max = i_h_max, inner_threshold_s_max = i_h_max, inner_threshold_v_max = i_v_max;
}

bool load_extrinsic_calibration(string yaml_path)
{
	try {
		YAML::Node yaml = YAML::LoadFile(yaml_path);

		double extrinsic_array[9];

		for(int i = 0; i < 9; i++) {
			extrinsic_array[i] = yaml["extrinsic_matrix"][i].as<double>();
		}

		H = (cv::Mat1d(3, 3) <<
			extrinsic_array[0], extrinsic_array[1], extrinsic_array[2],
			extrinsic_array[3], extrinsic_array[4], extrinsic_array[5],
			extrinsic_array[6], extrinsic_array[7], extrinsic_array[8]);

		cout << fixed << setprecision(5);
		cout << "homography matrix (extrinsic parameters):\n";
		for(int i = 0; i < 3; i++) {
			cout << "["  << extrinsic_array[i * 3 + 0]
		     	     << ", " << extrinsic_array[i * 3 + 1]
		     	     << ", " << extrinsic_array[i * 3 + 2] << "]\n";
		}
		return true;
	} catch(...) {
		return false;
	}
}

/* input a lane segment and recognize whether it is the left side
   or the right side of the road lane mark */
bool edge_side_detect(cv::Mat& threshold_image, segment_t& lane_segment)
{
        Point2f p1, p2;
        p1.x = lane_segment.untransformed.x1;
        p1.y = lane_segment.untransformed.y1;
        p2.x = lane_segment.untransformed.x2;
        p2.y = lane_segment.untransformed.y2;

	//swap if p1 is higher
	if(p1.y < p2.y) {
		Point2f tmp;
		tmp = p1;
		p1 = p2;
		p2 = tmp;
	}

	//vector normalization
	Point2f t_hat = p2 - p1;
	normalize(t_hat);

	//calculate the midpoint between p1 and p2
	Point2f midpoint = (p2 + p1) / 2;

	//dind the normal vector
	Point2f n_hat(-t_hat.y, t_hat.x);

	int left_cnt = 0, right_cnt = 0; //Left, right accumulator
	int x, y;

	for(int i = 0 ; i < SIDE_DETECT_PIXEL_CNT; i++) {
		/* test the pixels in positive direction */
		x = ceil(midpoint.x + n_hat.x * i);
		y = ceil(midpoint.y + n_hat.y * i);
		bound(0, IMAGE_WIDTH, x); //preventing from exceeding the image
		bound(0, IMAGE_HEIGHT / 2, y);
		if(threshold_image.at<uint8_t>(Point(x, y)) >= 255) {
			left_cnt++;
		}

		/* test the pixels in negitive direction */
		x = ceil(midpoint.x - n_hat.x * i);
		y = ceil(midpoint.y - n_hat.y * i);
		bound(0, IMAGE_WIDTH, x); //preventing from exceeding the image
		bound(0, IMAGE_HEIGHT / 2, y);
		if(threshold_image.at<uint8_t>(Point(x, y)) >= 255) {
			right_cnt++;
		}
	}

	//cout << ("left count:%d right count:%d\n", left_cnt, right_cnt);

	if(left_cnt > SIDE_DETECT_THREDHOLD && right_cnt > SIDE_DETECT_THREDHOLD) {
		lane_segment.side = UNKNOWN_SIDE;
		return false;
	}
	if(left_cnt < SIDE_DETECT_THREDHOLD && right_cnt < SIDE_DETECT_THREDHOLD) {
		lane_segment.side = UNKNOWN_SIDE;
		return false;
	}

	if(left_cnt > SIDE_DETECT_THREDHOLD) {lane_segment.side = LEFT_EDGE;}
	if(right_cnt > SIDE_DETECT_THREDHOLD) {lane_segment.side = RIGHT_EDGE;}

	return true;
}

/* recongnize all segment's side on the image */
void edge_side_detect_whole_image(vector<Vec4f>& opencv_segment_list,
			          vector<segment_t>& puyuma_segment_list,
				  cv::Mat& threshold_image)
{
	for(size_t i = 0; i < opencv_segment_list.size(); i++) {
		segment_t segment;

		segment.untransformed.x1 = opencv_segment_list[i][0];
		segment.untransformed.y1 = opencv_segment_list[i][1];
		segment.untransformed.x2 = opencv_segment_list[i][2];
		segment.untransformed.y2 = opencv_segment_list[i][3];

		edge_side_detect(threshold_image, segment);

		puyuma_segment_list.push_back(segment);
	}
}

/* visualization utilities */
void draw_segment_side(cv::Mat& lane_mark_image, vector<segment_t>& lane_segments)
{
	Point2f midpoint;

	for(size_t i = 0; i < lane_segments.size(); i++) {
		midpoint.x = (lane_segments.at(i).untransformed.x1 +
		lane_segments.at(i).untransformed.x2) / 2 + roi_offset_x;
		midpoint.y = (lane_segments.at(i).untransformed.y1 +
		lane_segments[i].untransformed.y2) / 2 + roi_offset_y;

		if(lane_segments.at(i).side == LEFT_EDGE) {
			putText(lane_mark_image, "l", midpoint, FONT_HERSHEY_COMPLEX_SMALL,
				1, Scalar(0, 0.7, 255));
		} else if(lane_segments.at(i).side == RIGHT_EDGE) {
			putText(lane_mark_image, "r", midpoint, FONT_HERSHEY_COMPLEX_SMALL,
				1, Scalar(0, 0.7, 255));
		}
	}
}

/* visualization utilities */
void draw_bird_view_image(cv::Mat& original_image, cv::Mat& bird_view_image)
{
	warpPerspective(original_image, bird_view_image, H, original_image.size());
}

void segments_homography_transform(vector<segment_t>& lines)
{
	for(size_t i = 0; i < lines.size(); i++) {
		vector<Point2f> segment;
		vector<Point2f> segment_transformed;

		Point2f point;
		point.x = lines.at(i).untransformed.x1 + roi_offset_x;
		point.y = lines.at(i).untransformed.y1 + roi_offset_y;
		segment.push_back(point);
		point.x = lines.at(i).untransformed.x2 + roi_offset_x;
		point.y = lines.at(i).untransformed.y2 + roi_offset_y;
		segment.push_back(point);

		perspectiveTransform(segment, segment_transformed, H);

		lines.at(i).x1 = segment_transformed.at(0).x;
		lines.at(i).y1 = segment_transformed.at(0).y;
		lines.at(i).x2 = segment_transformed.at(1).x;
		lines.at(i).y2 = segment_transformed.at(1).y;

		//cout << "p1(%f,%f) p2(%f,%f)", lines[i][0],
		//        lines[i][1], lines[i][2], lines[i][3];
	}
}

/* convert image size from pixel to centimeter */
void image_to_gnd(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y)
{
	gnd_x = pixel_x * (((BOARD_WIDTH + 2) * BOARD_BOX_SIZE) / IMAGE_WIDTH);
	gnd_y = pixel_y * (((BOARD_HEIGHT + 2) * BOARD_BOX_SIZE) / IMAGE_HEIGHT);
}

/* convert image size from centimeter to pixel */
void gnd_to_image(float& pixel_x, float& pixel_y, float& gnd_x, float& gnd_y)
{
	pixel_x = gnd_x * (IMAGE_WIDTH / ((BOARD_WIDTH + 2) * BOARD_BOX_SIZE));
	pixel_y = gnd_y * (IMAGE_HEIGHT / ((BOARD_HEIGHT + 2) * BOARD_BOX_SIZE));
}

void draw_region_of_interest(cv::Mat lane_mark_image)
{
	Point2f p1(0, roi_offset_y);
	Point2f p2(IMAGE_WIDTH, roi_offset_y);
	Point2f p3(IMAGE_WIDTH, IMAGE_HEIGHT);
	Point2f p4(0, IMAGE_HEIGHT);

	cv::line(lane_mark_image, p1, p2, Scalar(255, 128, 0), 2, CV_AA);
	cv::line(lane_mark_image, p2, p3, Scalar(255, 128, 0), 2, CV_AA);
	cv::line(lane_mark_image, p3, p4, Scalar(255, 128, 0), 2, CV_AA);
	cv::line(lane_mark_image, p4, p1, Scalar(255, 128, 0), 2, CV_AA);

	putText(lane_mark_image, "Region of interest",
		Point(roi_offset_x + 10, IMAGE_HEIGHT - 10),
		FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(255, 128, 0));
}

/* generate a lane segment vote for histogram filter */
bool generate_vote(segment_t& lane_segment, float& d, float& phi, int color)
{
	if(lane_segment.side == UNKNOWN_SIDE) {
		return false;
	}

	Point2f _p1, _p2;
	_p1.x = lane_segment.x1;
	_p1.y = lane_segment.y1;
	_p2.x = lane_segment.x2;
	_p2.y = lane_segment.y2;

	/* set new origin */
	_p1.x -= SEMI_IMAGE_WIDTH;
	_p2.x -= SEMI_IMAGE_WIDTH;

	Point2f p1, p2;
	image_to_gnd(_p1.x, _p1.y, p1.x, p1.y);
	image_to_gnd(_p2.x, _p2.y, p2.x, p2.y);

	/* swap if pi is higher */
	if(p1.y < p2.y) {
		Point2f tmp;
		tmp = p1;
		p1 = p2;
		p2 = tmp;
	}

	/* estimate d */
	Point2f t_hat = p2 - p1;
	normalize(t_hat);

	/* estimate phi */
	phi = atan2f(t_hat.y, t_hat.x) + PI / 2;

	Point2f n_hat(-t_hat.y, t_hat.x); //normal vector

	Point2f offset_vector = n_hat;

	float steady_bias = 4; //[cm]

	if(color == WHITE) {
		if(lane_segment.side == RIGHT_EDGE) {
			offset_vector *= -(W / 2) - L_W;
		} else {
			offset_vector *= -(W / 2);
		}
	} else if(color == YELLOW) {
		if(lane_segment.side == LEFT_EDGE) {
			offset_vector *= +(W / 2) + L_Y + steady_bias;
		} else {
			offset_vector *= +(W / 2) + steady_bias;
		}
	}

	p1 += offset_vector;
	p2 += offset_vector;

	p1.x -= CAMERA_TO_CENTER * sin(phi);
	p1.y -= CAMERA_TO_CENTER * cos(phi);
	p2.x -= CAMERA_TO_CENTER * sin(phi);
	p2.y -= CAMERA_TO_CENTER * cos(phi);

	float d1 = inner_product(n_hat, p1);
	float d2 = inner_product(n_hat, p2);

	d = (d1 + d2) / 2; //lateral displacement

	d *= -1;

	phi = rad_to_deg(phi);

	return true;
}

void lane_estimator_init()
{
	outer_threshold_h_min = 0; outer_threshold_h_max = 256;
	outer_threshold_s_min = 0; outer_threshold_s_max = 256;
	outer_threshold_v_min = 0; outer_threshold_v_max = 256;
	inner_threshold_h_min = 0; inner_threshold_h_max = 256;
	inner_threshold_s_min = 0; inner_threshold_s_max = 256;
	inner_threshold_s_min = 0; inner_threshold_s_max = 256;
	inner_threshold_v_min = 0; inner_threshold_v_max = 256;

	roi_offset_x = 0;
	roi_offset_y = IMAGE_HEIGHT / 2;
}

bool lane_estimate(cv::Mat& raw_image, float& final_d, float& final_phi)
{
	/* (step 1) feature extraction */

	//cut region of interest image
	cv::Mat roi_image;
        cv::Rect region(0, IMAGE_HEIGHT / 2, IMAGE_WIDTH, IMAGE_HEIGHT / 2);
        roi_image = raw_image(region);

	//convert image space from rgb to hsv
	cv::cvtColor(roi_image, outer_hsv_image, COLOR_BGR2HSV);
	cv::cvtColor(roi_image, inner_hsv_image, COLOR_BGR2HSV);

	//hsv color thresholding
	cv::inRange(
		outer_hsv_image,
		Scalar(outer_threshold_h_min, outer_threshold_s_min, outer_threshold_v_min),
		Scalar(outer_threshold_h_max, outer_threshold_s_max, outer_threshold_v_max),
		outer_threshold_image
	);
	cv::inRange(
		inner_hsv_image,
		Scalar(inner_threshold_h_min, inner_threshold_s_min, inner_threshold_v_min),
		Scalar(inner_threshold_h_max, inner_threshold_s_max, inner_threshold_v_max),
		inner_threshold_image
	);
	if(show_hsv_threshold_image == true) {
		imshow("outer hsv threshold image", outer_threshold_image);
		imshow("inner hsv threshold image", inner_threshold_image);
	}

	//canny edge detection
	cv::Mat outer_gray_image, inner_gray_image;
	cv::cvtColor(roi_image, outer_gray_image, CV_BGR2GRAY);
	cv::cvtColor(roi_image, inner_gray_image, CV_BGR2GRAY);
	cv::Mat preprocess_canny_image;
	cv::Canny(outer_gray_image, preprocess_canny_image, CANNY_THRESHOLD_1,
	          CANNY_THRESHOLD_2, 3);

	//image deliation on canny image
	Mat deliation_element = cv::getStructuringElement(MORPH_RECT, Size(3, 3));
	cv::dilate(preprocess_canny_image, canny_image, deliation_element);

	//doing bitwise AND on canny image hsv thresholding image)
	cv::Mat outer_bitwise_and_image, inner_bitwise_and_image;
	cv::bitwise_and(outer_threshold_image, canny_image, outer_bitwise_and_image);
	cv::bitwise_and(inner_threshold_image, canny_image, inner_bitwise_and_image);

	/* hough transform */
	vector<Vec4f> outer_cv_lines, inner_cv_lines;
	cv::HoughLinesP(outer_bitwise_and_image, outer_cv_lines, 1,
		        CV_PI / 180, HOUGH_THRESHOLD, 50, 5);
	cv::HoughLinesP(inner_bitwise_and_image, inner_cv_lines, 1, 
		        CV_PI / 180, HOUGH_THRESHOLD, 50, 5);

	/* convert opencv segment format to puyuma segment format and do segment
	   side recognization */
	vector<segment_t> outer_lines, inner_lines;
	edge_side_detect_whole_image(outer_cv_lines, outer_lines, outer_threshold_image);
	edge_side_detect_whole_image(inner_cv_lines, inner_lines, inner_threshold_image);

	if(outer_lines.size() == 0 && inner_lines.size() == 0) {
		//send_visualize_image(raw_image, canny_image, outer_threshold_image,
		//		     inner_threshold_image);
		cout << "failed to estimate the lane [no segment is found]\n";
		return false;
	}

	/* (step 2) apply homograpgy transform to the detected segments */

	segments_homography_transform(outer_lines);
	segments_homography_transform(inner_lines);

	/* (step 3) histogram filtering */

	int vote_count = 0; //initialize vote count of histogram filter
	
	/* 2d histogram, size = row * column */
	float vote_box[HISTOGRAM_R_SIZE][HISTOGRAM_C_SIZE] = {0.0f};

	/* generate votes */

	//for white land marks:
	for(size_t i = 0; i < outer_lines.size(); i++) {
		float d_i, phi_i;

		if(generate_vote(outer_lines.at(i), d_i, phi_i, WHITE) == false) {
			outer_lines.erase(outer_lines.begin() + i);
			continue;
		}

		outer_lines.at(i).d = d_i;
		outer_lines.at(i).phi = phi_i;

		vote_count++;

		//vote to histogram[_i][_j]
		int _i = (int)round((phi_i - PHI_MIN) / DELTA_PHI);
		int _j = (int)round((d_i - D_MIN) / DELTA_D);

		//drop the vote if it exceeded the boundary
		if(_i >= HISTOGRAM_R_SIZE || _j >= HISTOGRAM_C_SIZE) {
			outer_lines.erase(outer_lines.begin() + i) ;
			continue;	
		}

		vote_box[_i][_j] += 1.0f; //assume that every vote is equally important

		//XXX: publish vote info for debugging
	}

	//for yellow land marks:
	for(size_t i = 0; i < inner_lines.size(); i++) {
		float d_i, phi_i;

		if(generate_vote(inner_lines.at(i), d_i, phi_i, YELLOW) == false) {
			inner_lines.erase(inner_lines.begin() + i);
			continue;
		}

		inner_lines.at(i).d = d_i;
		inner_lines.at(i).phi = phi_i;

		vote_count++;

		//vote to histogram[_i][_j]
		int _i = (int)round((phi_i - PHI_MIN) / DELTA_PHI);
		int _j = (int)round((d_i - D_MIN) / DELTA_D);

		//drop the vote if it exceeded the boundary
		if(_i >= HISTOGRAM_R_SIZE || _j >= HISTOGRAM_C_SIZE) {
			inner_lines.erase(inner_lines.begin() + i);
			continue;	
		}

		vote_box[_i][_j] += 1.0; //assume that every vote is equally important

		//XXX: publish vote info for debugging
	}

	/* find the highest vote */
	int highest_vote_i = 0, highest_vote_j = 0;

	for(int i = 0; i < HISTOGRAM_R_SIZE; i++) {
		for(int j = 0; j < HISTOGRAM_C_SIZE; j++) {
			if(vote_box[i][j] > vote_box[highest_vote_i][highest_vote_j]) {
				//change the index to the winner
				highest_vote_i = i;
				highest_vote_j = j;
			}
		}
	}

	if(vote_box[highest_vote_i][highest_vote_j] < HISTOGRAM_FILTER_THRESHOLD) {
		//send_visualize_image(raw_image, canny_image, outer_threshold_image,
		//		     inner_threshold_image);
		cout << "failed to estimate the lane [less than threshold value]\n";
		return false;
	}

	/* convert i, j to most possible (phi,d) range */
	float predicted_phi = DELTA_PHI * highest_vote_i + PHI_MIN;
	float predicted_d = DELTA_D * highest_vote_j + D_MIN;
	//printf("Predicted phi:%f, d:%n", predicted_phi, predicted_d);

	/* phi and d should be in the range of predicted value +- delta/2 */
	float phi_up_bound = predicted_phi + DELTA_PHI / 2;
	float phi_low_bound = predicted_phi - DELTA_PHI / 2;
	float d_up_bound = predicted_d + DELTA_D / 2;
	float d_low_bound = predicted_d - DELTA_D / 2;

	float phi_mean = 0.0, d_mean = 0.0;
	int phi_sample_cnt = 0, d_sample_cnt = 0;

	//calculate the mean of the vote (TODO:weighted average?)
	for(size_t i = 0; i < inner_lines.size(); i++) {
		float phi_i, d_i;
		phi_i = inner_lines.at(i).phi;
		d_i = inner_lines.at(i).d;

		if(phi_i >= phi_low_bound && phi_i <= phi_up_bound) {
			phi_mean += phi_i;
			phi_sample_cnt++;
		}

		if(d_i >= d_low_bound && d_i <= d_up_bound) {
			d_mean += d_i;
			d_sample_cnt++;
		}
	}

	for(size_t i = 0; i < outer_lines.size(); i++) {
		float phi_i, d_i;
		phi_i = outer_lines.at(i).phi;
		d_i = outer_lines.at(i).d;

		if(phi_i >= phi_low_bound && phi_i <= phi_up_bound) {
			phi_mean += phi_i;
			phi_sample_cnt++;
		}

		if(d_i >= d_low_bound && d_i <= d_up_bound) {
			d_mean += d_i;
			d_sample_cnt++;
		}
	}

	if(phi_sample_cnt == 0 || d_sample_cnt == 0) {
		//send_visualize_image(raw_image, canny_image, outer_threshold_image,
		//		     inner_threshold_image);
		cout << "failed to estimate the lane [sample count equals zero]\n";
		return false;
	}

	phi_mean /= (float)phi_sample_cnt;
	d_mean /= (float)d_sample_cnt;

	final_d = d_mean;
	final_phi = phi_mean;

	//XXX: publish the final estimated (d, phi) info

	return true;
}
