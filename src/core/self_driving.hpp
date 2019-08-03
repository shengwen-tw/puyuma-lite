#ifndef __SELF_DRIVING_HPP__
#define __SELF_DRIVING_HPP__

#define THROTTLE_BASE 35 //35% of the throttle

typedef struct {
        float kp, ki, kd;
} pid_control_t;

void self_driving_control(float d, float phi);

void load_pid_param(string yaml_path);

#endif
