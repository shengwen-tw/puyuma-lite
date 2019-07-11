#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "lane_detector.hpp"

using namespace cv;

#define HSV_MIN 0
#define HSV_MAX 255

int i_h_min = HSV_MIN;
int i_h_max = HSV_MAX;
int i_s_min = HSV_MIN;
int i_s_max = HSV_MAX;
int i_v_min = HSV_MIN;
int i_v_max = HSV_MAX;

int o_h_min = HSV_MIN;
int o_h_max = HSV_MAX;
int o_s_min = HSV_MIN;
int o_s_max = HSV_MAX;
int o_v_min = HSV_MIN;
int o_v_max = HSV_MAX;

int save_color_calib = 0;

void save_color_calibration(std::string yaml_path)
{
	YAML::Node yaml_node;

	std::ofstream fout(yaml_path.c_str());

	YAML::Node inner_node = yaml_node["inner"];
	inner_node["h_min"] = i_h_min;
	inner_node["h_max"] = i_h_max;
	inner_node["s_min"] = i_s_min;
	inner_node["s_max"] = i_s_max;
	inner_node["v_min"] = i_v_min;
	inner_node["v_max"] = i_v_max;

	YAML::Node outer_node = yaml_node["outer"];

	outer_node["h_min"] = o_h_min;
	outer_node["h_max"] = o_h_max;
	outer_node["s_min"] = o_s_min;
	outer_node["s_max"] = o_s_max;
	outer_node["v_min"] = o_v_min;
	outer_node["v_max"] = o_v_max;

	fout << yaml_node; 
}

bool load_color_calibration(std::string yaml_path)
{
	try {
		YAML::Node yaml_node = YAML::LoadFile(yaml_path);

		YAML::Node inner_node = yaml_node["inner"];
		i_h_min = inner_node["h_min"].as<int>();
		i_h_max = inner_node["h_max"].as<int>();
		i_s_min = inner_node["s_min"].as<int>();
		i_s_max = inner_node["s_max"].as<int>();
		i_v_min = inner_node["v_min"].as<int>();
		i_v_max = inner_node["v_max"].as<int>();
		
		YAML::Node outer_node = yaml_node["outer"];
		o_h_min = outer_node["h_min"].as<int>();
		o_h_max = outer_node["h_max"].as<int>();
		o_s_min = outer_node["s_min"].as<int>();
		o_s_max = outer_node["s_max"].as<int>();
		o_v_min = outer_node["v_min"].as<int>();
		o_v_max = outer_node["v_max"].as<int>();

		set_outer_hsv_color_thresholding(o_h_min, o_s_min, o_v_min, o_h_max, o_s_max, o_v_max);
		set_inner_hsv_color_thresholding(i_h_min, i_s_min, i_v_min, i_h_max, i_s_max, i_v_max);

		return true;
	} catch(...) {
		return false;
	}
}

void color_setting_on_trackbar_callback(int state, void *_void)
{
	set_outer_hsv_color_thresholding(o_h_min, o_s_min, o_v_min, o_h_max, o_s_max, o_v_max);
	set_inner_hsv_color_thresholding(i_h_min, i_s_min, i_v_min, i_h_max, i_s_max, i_v_max);
}

void save_button_callback(int state, void *_void)
{
	cout << "save color calibration settings.\n";
	save_color_calibration("./color_calibration.yaml");
	exit(0);
}

void create_trackbars()
{
	namedWindow("outer hsv threshold image", 0);
	createTrackbar("H_MIN", "outer hsv threshold image", &o_h_min, 255, color_setting_on_trackbar_callback);
	createTrackbar("H_MAX", "outer hsv threshold image", &o_h_max, 255, color_setting_on_trackbar_callback);
	createTrackbar("S_MIN", "outer hsv threshold image", &o_s_min, 255, color_setting_on_trackbar_callback);
	createTrackbar("S_MAX", "outer hsv threshold image", &o_s_max, 255, color_setting_on_trackbar_callback);
	createTrackbar("V_MIN", "outer hsv threshold image", &o_v_min, 255, color_setting_on_trackbar_callback);
	createTrackbar("V_MAX", "outer hsv threshold image", &o_v_max, 255, color_setting_on_trackbar_callback);

	namedWindow("inner hsv threshold image", 0);
	createTrackbar("H_MIN", "inner hsv threshold image", &i_h_min, 255, color_setting_on_trackbar_callback);
	createTrackbar("H_MAX", "inner hsv threshold image", &i_h_max, 255, color_setting_on_trackbar_callback);
	createTrackbar("S_MIN", "inner hsv threshold image", &i_s_min, 255, color_setting_on_trackbar_callback);
	createTrackbar("S_MAX", "inner hsv threshold image", &i_s_max, 255, color_setting_on_trackbar_callback);
	createTrackbar("V_MIN", "inner hsv threshold image", &i_v_min, 255, color_setting_on_trackbar_callback);
	createTrackbar("V_MAX", "inner hsv threshold image", &i_v_max, 255, color_setting_on_trackbar_callback);

	namedWindow("Puyuma self-driving system", 0);
	createTrackbar("save calibration", "Puyuma self-driving system", &save_color_calib, 1, save_button_callback);
}

void hsv_color_thresholding_calibration(void)
{
	load_color_calibration("./color_calibration.yaml");
	create_trackbars();
	set_color_calib_mode();
}
