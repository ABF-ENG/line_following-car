#ifndef _MOTOR_BASE_H
#define _MOTOR_BASE_H

#include "stm32f4xx_hal.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;   /* PWM   */
extern TIM_HandleTypeDef htim2;   /* 左编码器（32 位） */
extern TIM_HandleTypeDef htim3;   /* 右编码器（16 位） */
extern volatile uint8_t g_ctrl_tick;

#define ENC_COUNTS_PER_REV   1560.0f    /* 手转一圈编码器实测值 */
#define WHEEL_DIAM_MM          65.0f    /* 轮子直径 */
#define CTRL_PERIOD_MS         10.0f    /* 控制周期，和 TIM6 一致 */
#define MM_PER_COUNT  (3.14159265f * WHEEL_DIAM_MM / ENC_COUNTS_PER_REV) //每一个脉冲对应多少毫米
#define MM_S_PER_CNT  (MM_PER_COUNT * (1000.0f / CTRL_PERIOD_MS))        //换算mm/s

static void set_pwm(uint32_t ch, int duty);
void Motor_Init(void);
void Motor_SetSpeed(int duty_l, int duty_r);
int32_t Motor_GetDeltaL(void);
int32_t Motor_GetDeltaR(void);
float Motor_GetSpeedL(void);
float Motor_GetSpeedR(void);
float Motor_GetDistL(void);
float Motor_GetDistR(void);

#endif