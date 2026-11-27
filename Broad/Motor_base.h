#ifndef _MOTOR_BASE_H
#define _MOTOR_BASE_H

#include "stm32f4xx_hal.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;   /* PWM   */
extern TIM_HandleTypeDef htim2;   /* 左编码器（32 位） */
extern TIM_HandleTypeDef htim3;   /* 右编码器（16 位） */
extern volatile uint8_t g_ctrl_tick;

/* GMR 编码器 500 线 x 减速比 28 x 4 倍频 = 56000 counts/轮子一圈
 * （手转实测 ~56500，误差 ~0.9% 来自圈数判读，规格值是精确的，取规格值） */
#define ENC_COUNTS_PER_REV  56000.0f
#define WHEEL_DIAM_MM          65.0f    /* 轮子直径 */
#define CTRL_PERIOD_MS         10.0f    /* 控制周期，和 TIM6 一致（PSC=83,ARR=9999 -> 84MHz/84/10000 = 10ms） */
#define MM_PER_COUNT  (3.14159265f * WHEEL_DIAM_MM / ENC_COUNTS_PER_REV) //每一个脉冲对应多少毫米
#define MM_S_PER_CNT  (MM_PER_COUNT * (1000.0f / CTRL_PERIOD_MS))        //换算mm/s

void Motor_Init(void);
void Motor_SetSpeed(int duty_l, int duty_r);
int32_t Motor_GetDeltaL(void);
int32_t Motor_GetDeltaR(void);
float Motor_GetSpeedL(void);
float Motor_GetSpeedR(void);
float Motor_GetDistL(void);
float Motor_GetDistR(void);

/* ---- 急停（PC0，按下 = 低电平，短路刹车 + 锁存） ---- */
uint8_t Motor_IsEStopPressed(void);   /* 1 = 按键当前按着 */
uint8_t Motor_IsEStopLatched(void);   /* 1 = 已锁存急停（松手也不恢复） */
void    Motor_ClearEStop(void);       /* 清除锁存（需先松手） */

#endif