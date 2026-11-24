#ifndef _GRAY_H
#define _GRAY_H

#include "stm32f4xx_hal.h"

/*
 * 八路灰度模块驱动（CD4051 复用的数字量输出）
 *
 * 接线（扩展板 J? 灰度口）：
 *      模块 5V   → 5V
 *      模块 GND  → GND
 *      模块 AD0  → PD0    地址 bit0
 *      模块 AD1  → PD1    地址 bit1
 *      模块 AD2  → PD2    地址 bit2
 *      模块 OUT  → PD3    数字量输出
 *
 * 通道与时序：
 *      AD2 AD1 AD0 = 000 → OUT 上是 X1，001 → X2，… 111 → X8
 *      每次切换地址后要等 GRAY_SETTLE_US 再读（模块内部比较器 + 输出电容的建立时间）
 *
 * 返回的 mask 位序：
 *      bit0 = X1, bit1 = X2, … bit7 = X8
 *      1 = 读到高电平，0 = 读到低电平
 *      ★ 高低电平对应"白"还是"黑"，不同版本模块可能相反，必须实测（见 Gray_ReadMask 注释）
 */

/* ---- 引脚映射：CubeMX 里改了脚只改这一段 ---- */
#define GRAY_GPIO_ADDR    GPIOD
#define GRAY_AD0_PIN      GPIO_PIN_0
#define GRAY_AD1_PIN      GPIO_PIN_1
#define GRAY_AD2_PIN      GPIO_PIN_2

#define GRAY_GPIO_OUT     GPIOD
#define GRAY_OUT_PIN      GPIO_PIN_3

void    Gray_Init(void);

/* 读一次全部 8 路，返回 8 位 mask。耗时 ≈ 8 × GRAY_SETTLE_US，默认 400µs */
uint8_t Gray_ReadMask(void);

/* 把 mask 转成可读字符串 "10010000"，out 至少 9 字节。调试打印用 */
void    Gray_MaskToStr(uint8_t mask, char *out);

#endif
