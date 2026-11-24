#ifndef _TRACE_H
#define _TRACE_H

#include "stm32f4xx_hal.h"

/*
 * 循迹决策层（只算"该跑多快"，不直接碰电机）
 *
 * 数据流：
 *     Gray_ReadMask()  →  本层算误差/转向  →  MotorAdvance_SetTarget()
 *                                              ↓
 *                                        MotorAdvance_Tick()  速度环下发 PWM
 *
 * 极性约定（实测确认）：mask 对应位 = 1 表示该探头压在黑线上，0 = 白底
 *
 * 用法（main.c）：
 *     Trace_Init();
 *     Trace_Start();                 // 声控/按键触发后调
 *
 *     while (1)
 *     {
 *         if (g_ctrl_tick)
 *         {
 *             g_ctrl_tick = 0;
 *             Trace_Tick();          // 读灰度 → 设定目标速度
 *             MotorAdvance_Tick();   // 速度环 → 下发 PWM
 *         }
 *     }
 */

typedef enum
{
    TRACE_IDLE = 0,     /* 未发车 / 已停车 */
    TRACE_RUN,          /* 循迹中 */
    TRACE_FINISH        /* 判定到终点 */
} TraceState;

void    Trace_Init(void);
void    Trace_Start(void);
void    Trace_Stop(void);

/* 每 20ms 调一次，内部读灰度并设置左右轮目标速度 */
void    Trace_Tick(void);

TraceState Trace_GetState(void);
uint8_t    Trace_GetMask(void);
float      Trace_GetError(void);        /* 负 = 线在左，正 = 线在右，量纲约 = 探头间距 */
float      Trace_GetSteer(void);        /* 本轮转向量 mm/s */
int16_t    Trace_GetTargetL(void);
int16_t    Trace_GetTargetR(void);

/* ---- 调试镜像（Watch 不能调函数，看这些变量）----
 *   g_trace_mask   当前 8 位灰度
 *   g_trace_err    当前误差（正常循迹：-7~+7；盲走时：航向误差，度）
 *   g_trace_steer  当前转向量 mm/s
 *   g_trace_tgtl/tgtr 左右轮目标速度
 *   g_trace_blind  1 = 正在盲走
 *   g_trace_ye     盲走时的航向误差，度
 */
extern volatile uint8_t g_trace_mask;
extern volatile float   g_trace_err;
extern volatile float   g_trace_steer;
extern volatile int16_t g_trace_tgtl;
extern volatile int16_t g_trace_tgtr;
extern volatile uint8_t g_trace_blind;
extern volatile float   g_trace_ye;
extern volatile uint8_t g_trace_cross;   /* 已压到几次全黑线 */
extern volatile uint8_t g_trace_fin;     /* 1 = 正在终点冲刺 */
extern volatile uint8_t g_trace_ab;      /* 1 = 盲走已出来，正在计 60cm */
extern volatile uint8_t g_trace_edge;    /* X8 边灯已亮过几次 */
extern volatile uint8_t g_trace_ft;      /* 1 = 正在强制转向 */

#endif
