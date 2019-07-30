#ifndef __MOTOR_HPP__
#define __MOTORR_HPP__

#include "string.h"

#define MOTOR_PWM_MAX 100
#define MOTOR_PWM_MIN 0

using namespace std;

void load_motor_calibration(string yaml_path);
void set_motor_pwm(int8_t left_pwm, int8_t right_pwm);
void motor_init();
void test_motor();
void halt_motor();

#endif
