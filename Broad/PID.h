#ifndef  _PID_H
#define  _PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;
    float error;
    float i;
    float derivative;
    float i_Max;
}PID;

void PID_init(PID *pid, float Kp, float Ki, float Kd, float target);
void PID_Reset(PID *pid);
#endif