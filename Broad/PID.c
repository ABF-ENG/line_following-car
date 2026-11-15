#include "PID.h"
void PID_init(PID *pid, float Kp, float Ki, float Kd, float target)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = target;
    pid->error = 0.0f;
    pid->i= 0.0f;
    pid->derivative = 0.0f;
    pid->i_Max = 100.0f; // Set a default maximum value for the integral term
}

void PID_Reset(PID *pid)
{
    pid->error = 0.0f;
    pid->i = 0.0f;
    pid->derivative = 0.0f;
}

