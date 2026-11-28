#ifndef _MPU6050_H
#define _MPU6050_H

#include "stm32f4xx_hal.h"

/*
 * MPU6050 陀螺仪驱动 —— 只为"盲走保持航向"这一个用途
 *
 * 接线（扩展板）：SCL = PB6, SDA = PB7, I2C1
 * 只用陀螺仪的 Z 轴（垂直轴），加速度计暂时不用。
 *
 * 典型用法：
 *     Mpu_Init();          // 上电
 *     ...                  // 车摆好、完全静止
 *     Mpu_Calibrate();     // 发车前校准零偏（阻塞 1 秒，车必须静止！）
 *
 *     // 每 20ms 调一次
 *     Mpu_Update();
 *
 *     // 航向保持
 *     float err = Mpu_GetYaw() - yaw_ref;
 *     steer = KP_YAW * err;
 */

#define MPU_OK      0u
#define MPU_ERR     1u

/* 初始化：自动探测地址 0x68 / 0x69，校验 WHO_AM_I。返回 MPU_OK 表示成功 */
uint8_t Mpu_Init(void);

/* 静止校准零偏：采样 200 次（约 1 秒），期间车必须完全静止 */
void    Mpu_Calibrate(void);

/* 每 20ms 调一次：读陀螺仪三轴 + 积分出航向角 */
void    Mpu_Update(void);

/* 把航向角清零 / 设成指定值 */
void    Mpu_ResetYaw(void);

/* ---- 观测 ---- */
float   Mpu_GetYaw(void);       /* 积分出的航向角，度，±180 */
float   Mpu_GetYawRate(void);   /* 扣除零偏后的角速度 °/s —— 阻尼用这个 */
float   Mpu_GetBiasZ(void);     /* 校准得到的零偏 */
float   Mpu_GetGX(void);        /* 三轴原始角速度，用于确认哪个轴是垂直轴 */
float   Mpu_GetGY(void);
float   Mpu_GetGZ(void);
uint8_t Mpu_IsReady(void);

/* ---- 验证用：静置采 n 次，统计原始 gz 的均值/最小/最大 ----
 * 在 Mpu_Calibrate() 之前调用，看到的就是真实零偏和噪声。
 * 结果在下面三个变量里，Keil Watch 直接加即可。 */
void Mpu_NoiseSweep(uint16_t n);
extern volatile float g_mpu_mean;
extern volatile float g_mpu_min;
extern volatile float g_mpu_max;

/* ---- 调试用全局镜像 ----
 * Keil 的 Watch 不能调函数（会显示 <cannot evaluate>），所以看这些变量：
 *     g_mpu_ok      1 = 初始化成功
 *     g_mpu_who     读到的 WHO_AM_I（0 = I2C 完全不通）
 *     g_mpu_addr7   有响应的 7 位地址（0x68 / 0x69）
 *     g_mpu_i2c_spd 实际用的速率 kHz（400 或 100）
 *     g_mpu_scan_cnt / g_mpu_scan_first  扫描结果
 *     g_mpu_gx / g_mpu_gy / g_mpu_gz   三轴角速度 °/s
 *     g_mpu_yaw     航向角 度
 *     g_mpu_rate    扣零偏后的 Z 轴角速度 °/s
 *     g_mpu_bias    零偏 °/s
 */
extern volatile uint8_t  g_mpu_ok;
extern volatile uint8_t  g_mpu_who;
extern volatile uint8_t  g_mpu_addr7;
extern volatile uint16_t g_mpu_i2c_spd;
extern volatile uint8_t  g_mpu_scan_cnt;
extern volatile uint8_t  g_mpu_scan_first;
extern volatile float    g_mpu_gx;
extern volatile float    g_mpu_gy;
extern volatile float    g_mpu_gz;
extern volatile float    g_mpu_yaw;
extern volatile float    g_mpu_rate;
extern volatile float    g_mpu_bias;
extern volatile uint8_t  g_mpu_bias_adapt;   /* 1 = 零偏自动跟踪（温漂补偿），串口 "j0" 关闭 */

/* 扫描总线 0x08~0x77，结果放在 g_mpu_scan_cnt / g_mpu_scan_first。
 * g_mpu_ok = 0 且 g_mpu_who = 0 时调用它，区分"没器件"和"地址不对"。 */
void Mpu_ScanBus(void);

#endif
