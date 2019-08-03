#include <yaml-cpp/yaml.h>

#include "common.hpp"
#include "motor.hpp"
#include "lane_detector.hpp"
#include "self_driving.hpp"

pid_control_t pid_d;
pid_control_t pid_phi;

void load_pid_param(string yaml_path)
{
	try {
		YAML::Node yaml = YAML::LoadFile(yaml_path);

		pid_d.kp = yaml["pid_d"]["kp"].as<float>();
		pid_d.ki = yaml["pid_d"]["ki"].as<float>();
		pid_d.kd = yaml["pid_d"]["kd"].as<float>();
		pid_phi.kp = yaml["pid_phi"]["kp"].as<float>();
		pid_phi.ki = yaml["pid_phi"]["ki"].as<float>();
		pid_phi.kd = yaml["pid_phi"]["kd"].as<float>();

		cout << "[PID d controller]\n" <<
			"Kp: " << pid_d.kp <<  ", Ki: " << pid_d.ki << ", Kd:" << pid_d.kd <<
			"[PID phi controller]\n" <<
			"Kp: " << pid_phi.kp <<  ", Ki: " << pid_phi.ki << ", Kd:" << pid_phi.kd;
	} catch(...) {
		cout << "failed to load PID parameters, load default.\n" <<
			"[PID d controller]\n" <<
			"Kp: 0, Ki: 0, Kd: 0\n" <<
			"[PID phi controller]\n" <<
			"Kp: 0, Ki: 0, Kd: 0\n";

		pid_d.kp = 0;
		pid_d.ki = 0;
		pid_d.kd = 0;
		pid_phi.kp = 0;
		pid_phi.ki = 0;
		pid_phi.kd = 0;
	}
}

static float pid_control(int current_value, float setpoint, pid_control_t& pid)
{
	//a p controller is already good enough for this case
	return pid.kp * (current_value - setpoint);
}

/* the puyuma self-driving algorithm is a cascaded p controller */
void self_driving_control(float d, float phi)
{
	int pwm_left, pwm_right;

	float d_setpoint = -2.0f; //[cm]
	float phi_setpoint = pid_control(d, d_setpoint, pid_d);

	bound(PHI_MIN, PHI_MAX, phi_setpoint);

	//cascading d controller with phi, controlling the d by changing phi setpoint
	int pwm = (int)pid_control(phi, phi_setpoint, pid_phi);

	pwm_left = THROTTLE_BASE + pwm;
	pwm_right = THROTTLE_BASE - pwm;

	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_left);
	bound(MOTOR_PWM_MIN, MOTOR_PWM_MAX, pwm_right);

	set_motor_pwm(pwm_left, pwm_right);
}

