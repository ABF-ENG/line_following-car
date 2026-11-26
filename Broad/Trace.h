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
extern volatile uint8_t g_trace_lev;     /* 第几次丢线（诊断：盲走应该是第 8 次） */
extern volatile float   g_trace_dist;    /* 累计里程 mm（诊断用） */

/* ---- 运行时可改的调参量（串口命令 p/d/b/s/l/y/g/m 可写）---- */
extern float g_base_mm_s;       /* 循迹基础车速 mm/s */
extern float g_kp_trace;        /* 循迹比例增益 */
extern float g_kd_trace;        /* 循迹微分增益 */
extern float g_search_steer;    /* 丢线找线差速 mm/s */
extern float g_blind_speed;     /* 盲走速度 mm/s */
extern float g_kp_yaw;          /* 盲走航向增益 mm/s per ° */
extern float g_kd_yaw_rate;     /* 航向速率阻尼 (mm/s)/(°/s)，0 = 关闭 */
extern float g_steer_limit;     /* 航向修正差速限幅 mm/s */
extern float g_blind_offset;    /* 盲走目标航向偏移角 度 */
extern float g_island_turn_speed;  /* 环岛强制转向速度 mm/s */
extern float g_island_turn_deg;    /* 环岛强制转向角度 度 */
extern float g_lpf_alpha;          /* 误差低通系数，1.0 = 关闭 */
extern float g_island_ramp_mm;     /* 环岛转向后速度渐变距离 mm */
extern float g_slow_start_mm;      /* 第5个直角后再走多少 mm 开始减速 */
extern float g_slow_speed;         /* 盲走前的慢速 mm/s */
extern uint16_t g_loss_debounce_ms; /* 丢线计数消抖时间 ms */
extern float g_corner6_gap_mm;     /* 第6个弯到第7个弯的最小间距 mm */

#endif
