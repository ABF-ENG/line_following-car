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
}PID_t;

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float target);
void PID_Reset(PID_t *pid);
#endif