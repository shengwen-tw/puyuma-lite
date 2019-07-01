#include "common.hpp"
#include "motor.hpp"
#include "lane_detector.hpp"
#include "self_driving.hpp"

pid_control_t pid_d;
pid_control_t pid_phi;

static float pid_control(int current_value, float setpoint, pid_control_t& pid)
{
	//a p controller is already good enough for this case
	return pid.kp * (current_value - setpoint);
}

/* the puyuma self-driving algorithm is a cascaded p controller */
void self_driving_control(float d, float phi)
{
	int pwm_left, pwm_right;

	float phi_setpoint = pid_control(d, 0, pid_d);

	bound(PHI_MIN, PHI_MAX, phi_setpoint);

	//cascading d controller with phi, controlling the d by changing phi setpoint
	int pwm = (int)pid_control(phi, phi_setpoint, pid_phi);

	pwm_left = THROTTLE_BASE + pwm;
	pwm_right = THROTTLE_BASE - pwm;

	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_left);
	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_right);

        //set_motor_pwm(pwm_left, pwm_right);
}

