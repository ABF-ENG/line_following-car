#ifndef _DEBUG_UART_H
#define _DEBUG_UART_H

#include "stm32f4xx_hal.h"

/*
 * USART2 上的 CSV 调试输出，配合 SerialPlot 看波形。
 * 引脚：PA2 = TX, PA3 = RX（板上 U19 的 3/4 脚），115200 8N1。
 *
 * 输出格式（20Hz，每行 13 列，逗号分隔）：
 *     #tgtL,spdL,dtyL,tgtR,spdR,dtyR,errx10,steer,mask,yawx10,ratex10,mode,yex10
 *      500, 495,  33, 500, 498,  34,    12,   45,    24,    35,    -2,    0,   0
 *
 *   tgtL/R    左/右轮目标速度 mm/s（来自速度环）
 *   spdL/R    左/右轮实测速度 mm/s
 *   dtyL/R    左/右轮实际占空比 %
 *   errx10    循迹误差 ×10（原始 −7…+7，放大 10 倍方便看）
 *   steer     转向量 mm/s（循迹 = PD 输出；盲走/环岛 = 航向环输出）
 *   mask      八路灰度原值（bit0 = X1）—— 只看不画，或当参考
 *   yawx10    陀螺仪航向角 ×10（度）
 *   ratex10   角速度 ×10（°/s）
 *   mode      0=循迹 1=盲走 2=环岛转 3=终点冲刺 4=已停 5=手动
 *   yex10     **航向误差 ×10（度）** —— 盲走/环岛时才非零，
 *             盲走时它应该被拉回 0；一直变大说明 KP_YAW 太小或戴反了
 *
 * 发送是**非阻塞**的：每行先格式化进缓冲区，再由 Dbg_Task 逐字节喂给 UART，
 * 不会拖慢 10ms 控制节拍。
 *
 * ── 串口下行命令（运行中改参数，不用重新编译）──
 *   格式：字母 + 数值，后面加回车换行；不加也行，停 120ms 自动生效。
 *
 *     p38      KP_TRACE           循迹比例增益
 *     d15      KD_TRACE           循迹微分增益（支持小数，如 d12.5）
 *     b400     BASE_MM_S          循迹基础车速
 *     s340     SEARCH_STEER_MM_S  丢线找线差速
 *     l500     BLIND_SPEED_MM_S   盲走速度
 *     y7       KP_YAW             盲走航向增益
 *     m300     STEER_LIMIT_MM_S   航向修正差速限幅
 *     g180     BLIND_OFFSET_DEG   盲走目标航向偏移角
 *
 *     300      手动模式：左右轮都跑 300 mm/s（Trace 不再覆盖目标）
 *     0        停车（仍在手动模式）
 *     c        退出手动，恢复循迹（会重新 Trace_Start）
 */

void    Dbg_Init(void);

/* 主循环里每轮调一次（不阻塞，内部自己限速） */
void    Dbg_Task(void);

void    Dbg_Enable(uint8_t on);

/* 1 = 手动速度模式（Trace 不参与） */
uint8_t Dbg_IsManual(void);

#endif
