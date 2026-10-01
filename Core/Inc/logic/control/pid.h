#ifndef INC_PID_H_
#define INC_PID_H_


#include "global.h"
#include "params.h"

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
} PID_t;

void PID_Reset(PID_t *pid);

float PID_Update(PID_t *pid, float error, float dt, float output_min, float output_max);

#endif
