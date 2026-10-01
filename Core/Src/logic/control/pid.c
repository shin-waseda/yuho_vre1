#include "logic/control/pid.h"

void PID_Reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Update(PID_t *pid, float error, float dt, float output_min, float output_max) {
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output <= output_max && output >= output_min) {
        pid->integral += error * dt;
        output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    }

    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    return output;
}
