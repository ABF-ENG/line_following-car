#include "PID.h"
void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float target)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = target;
    pid->error = 0.0f;
    pid->i= 0.0f;
    pid->derivative = 0.0f;
    pid->i_Max = 100.0f; 
}

void PID_Reset(PID_t *pid)
{
    pid->error = 0.0f;
    pid->i = 0.0f;
    pid->derivative = 0.0f;
}

