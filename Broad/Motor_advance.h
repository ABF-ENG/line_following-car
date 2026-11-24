#ifndef _MOTOR_ADVANCE_H
#define _MOTOR_ADVANCE_H

#include "stm32f4xx_hal.h"

/*
 * 双轮速度环（前馈 + PI），控制周期 = TIM6 节拍 = 20ms
 *
 * 结构（每个轮子一套，互相独立）：
 *
 *   目标 mm/s ──> 斜坡限速 ──> ┬──────────────────────────┐
 *                             │ 前馈: v/FF_SPEED_AT_FULL │
 *   实测 mm/s <── 编码器 ──>  ┴─ PI 修残差 ──> 限幅 ──> PWM %
 *
 * 为什么要前馈：电机"占空比 → 转速"基本是线性的，先把大头算出来，
 * PI 只负责修电池掉压、地面摩擦、左右个体差异这些小量。
 * 没有前馈的话 Ki 必须开很大才能把稳态误差拉回来，容易震荡。
 *
 * 用法（main.c）：
 *
 *     Motor_Init();
 *     MotorAdvance_Init();
 *     MotorAdvance_SetTarget(500, 500);      // mm/s，负值 = 后退
 *
 *     while (1)
 *     {
 *         if (g_ctrl_tick) { g_ctrl_tick = 0; MotorAdvance_Tick(); }
 *     }
 *
 * 注：想彻底消掉主循环带来的下发抖动，可以把 MotorAdvance_Tick()
 *     直接挪进 Motor_base.c 的 HAL_TIM_PeriodElapsedCallback 里，
 *     紧跟在 Motor_EncoderUpdate() 后面，那样就是严格的 20ms 一拍。
 */

void MotorAdvance_Init(void);

/* 设定目标速度，单位 mm/s，带符号（负 = 后退）。正负号按"车前进"为正 */
void MotorAdvance_SetTarget(int16_t mm_s_l, int16_t mm_s_r);

/* 一个控制步：读实测 -> 算 PI -> 下发 PWM。每 20ms 调一次，不要更频繁 */
void MotorAdvance_Tick(void);

/* 停车：目标清零、积分清零、PWM 归零（滑行，不是刹车） */
void MotorAdvance_Stop(void);

/* 观测用（Keil Watch 里加这几条看波形） */
int16_t MotorAdvance_GetSpeedL(void);    /* 实测 mm/s */
int16_t MotorAdvance_GetSpeedR(void);
int16_t MotorAdvance_GetDutyL(void);     /* 实际下发的占空比 % */
int16_t MotorAdvance_GetDutyR(void);
int16_t MotorAdvance_GetTargetL(void);   /* 斜坡后的目标 mm/s */
int16_t MotorAdvance_GetTargetR(void);

/* 堵转保护：占空比给到 60% 以上、速度还低于 50mm/s，连续 500ms -> 置位并切断输出 */
uint8_t MotorAdvance_IsStalled(void);
void    MotorAdvance_ClearFault(void);   /* 故障复位（重新允许输出） */

#endif
