/**
  ******************************************************************************
  * @file    Mpu6050.c
  * @brief   MPU6050 陀螺仪驱动（只做航向角，供盲走路段保持直行）
  *
  *  为什么只积分不融合：
  *    MPU6050 没有磁力计，yaw 没有任何外部参考可以校正，
  *    所以只能靠"静止零偏校准 + 短时间积分"。盲走 1~2 秒内漂移 1~3° 可接受。
  *
  *  配置要点：
  *    - 陀螺仪量程 ±500°/s（原地转 90° 角速度能到 200~300°/s，±250 会削顶）
  *    - DLPF 20Hz：比 42Hz 更能压电机振动，避免 50Hz 采样时混叠
  *    - 时钟源用 PLL（比内部 RC 稳）
  *
  *  调试变量（Keil Watch 直接加，Watch 不能调函数）：
  *    g_mpu_ok      1 = 初始化成功
  *    g_mpu_who     读到的 WHO_AM_I（0 = 一个字节都没读到 → I2C 不通）
  *    g_mpu_addr7   有响应的 7 位地址（0x68 / 0x69）
  *    g_mpu_i2c_spd 实际用的速率（400 或 100，kHz）
  *    g_mpu_scan_cnt / g_mpu_scan_first  扫描总线上有几个器件、第一个是谁
  *    g_mpu_gx/gy/gz, g_mpu_yaw, g_mpu_rate, g_mpu_bias
  ******************************************************************************
  */

#include "Mpu6050.h"
#include "i2c.h"

/* ---------------- 寄存器 ---------------- */
#define MPU_REG_SMPLRT_DIV      0x19u
#define MPU_REG_CONFIG          0x1Au
#define MPU_REG_GYRO_CONFIG     0x1Bu
#define MPU_REG_ACCEL_CONFIG    0x1Cu
#define MPU_REG_GYRO_XOUT_H     0x43u
#define MPU_REG_PWR_MGMT_1      0x6Bu
#define MPU_REG_WHO_AM_I        0x75u

/* ±500°/s 时：65.5 LSB 表示 1 °/s */
#define MPU_GYRO_LSB_PER_DPS    65.5f

#define MPU_I2C_TIMEOUT         20u
#define MPU_DT_NOMINAL          0.02f

/* ---------------- 内部状态 ---------------- */

static uint16_t s_addr    = 0u;
static uint8_t  s_ready   = 0u;

static float    s_bias_z  = 0.0f;
static float    s_yaw     = 0.0f;

static float    s_gx      = 0.0f;
static float    s_gy      = 0.0f;
static float    s_gz      = 0.0f;
static float    s_rate_z  = 0.0f;

static uint32_t s_last_ms = 0u;

/* ---------------- 调试全局 ---------------- */

volatile uint8_t g_mpu_ok        = 0u;
volatile uint8_t g_mpu_who       = 0u;
volatile uint8_t g_mpu_addr7     = 0u;
volatile uint16_t g_mpu_i2c_spd  = 0u;
volatile uint8_t g_mpu_scan_cnt  = 0u;
volatile uint8_t g_mpu_scan_first = 0u;

volatile float   g_mpu_gx    = 0.0f;
volatile float   g_mpu_gy    = 0.0f;
volatile float   g_mpu_gz    = 0.0f;
volatile float   g_mpu_yaw   = 0.0f;
volatile float   g_mpu_rate  = 0.0f;
volatile float   g_mpu_bias  = 0.0f;

volatile float   g_mpu_mean  = 0.0f;
volatile float   g_mpu_min   = 0.0f;
volatile float   g_mpu_max   = 0.0f;

/* ---------------- I2C 小工具（带重试） ---------------- */

static uint8_t mpu_write8(uint8_t reg, uint8_t val)
{
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        if (HAL_I2C_Mem_Write(&hi2c1, s_addr, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1u,
                              MPU_I2C_TIMEOUT) == HAL_OK)
        {
            return MPU_OK;
        }
        HAL_Delay(1);
    }
    return MPU_ERR;
}

static uint8_t mpu_read8(uint8_t reg, uint8_t *val)
{
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        if (HAL_I2C_Mem_Read(&hi2c1, s_addr, reg,
                             I2C_MEMADD_SIZE_8BIT, val, 1u,
                             MPU_I2C_TIMEOUT) == HAL_OK)
        {
            return MPU_OK;
        }
        HAL_Delay(1);
    }
    return MPU_ERR;
}

static uint8_t mpu_read_bytes(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        if (HAL_I2C_Mem_Read(&hi2c1, s_addr, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len,
                             MPU_I2C_TIMEOUT) == HAL_OK)
        {
            return MPU_OK;
        }
        HAL_Delay(1);
    }
    return MPU_ERR;
}

static uint8_t mpu_read_gyro_raw(float *gx, float *gy, float *gz)
{
    uint8_t b[6];

    if (mpu_read_bytes(MPU_REG_GYRO_XOUT_H, b, 6u) != MPU_OK)
    {
        return MPU_ERR;
    }

    *gx = ((float)(int16_t)(((uint16_t)b[0] << 8) | b[1])) / MPU_GYRO_LSB_PER_DPS;
    *gy = ((float)(int16_t)(((uint16_t)b[2] << 8) | b[3])) / MPU_GYRO_LSB_PER_DPS;
    *gz = ((float)(int16_t)(((uint16_t)b[4] << 8) | b[5])) / MPU_GYRO_LSB_PER_DPS;

    return MPU_OK;
}

/* PB6/PB7 补内部上拉。
 * CubeMX 生成的是 GPIO_NOPULL，完全靠模块上的外部上拉；
 * 扩展板这条线比较长，补个弱上拉没有坏处。 */
static void mpu_gpio_pullup(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
}

/* 在 0x68/0x69 上找器件；找到返回 1 */
static uint8_t mpu_probe(uint8_t *who_out)
{
    static const uint16_t cand[2] = { (0x68u << 1), (0x69u << 1) };
    uint8_t  who;
    uint8_t  k;

    for (k = 0u; k < 2u; k++)
    {
        s_addr = cand[k];
        if (mpu_read8(MPU_REG_WHO_AM_I, &who) == MPU_OK)
        {
            *who_out     = who;
            g_mpu_addr7  = (uint8_t)(cand[k] >> 1);
            return 1u;
        }
    }
    return 0u;
}

/* ---------------- 对外接口 ---------------- */

uint8_t Mpu_Init(void)
{
    uint8_t who   = 0u;
    uint8_t found = 0u;
    uint8_t dummy = 0u;

    s_ready     = 0u;
    g_mpu_ok    = 0u;
    g_mpu_who   = 0u;
    g_mpu_addr7 = 0u;

    mpu_gpio_pullup();

    /* 先在 400kHz 下探测 */
    g_mpu_i2c_spd = (uint16_t)(hi2c1.Init.ClockSpeed / 1000u);
    found = mpu_probe(&who);

    if (found == 0u)
    {
        /* 400kHz 不通：降到 100kHz 再试一次（线长 / 上拉偏弱时的常见情况） */
        hi2c1.Init.ClockSpeed = 100000u;
        (void)HAL_I2C_Init(&hi2c1);
        g_mpu_i2c_spd = 100u;

        found = mpu_probe(&who);
    }

    if (found == 0u)
    {
        g_mpu_who = 0u;         /* 一个字节都没读到 → I2C 根本不通 */
        return MPU_ERR;
    }

    g_mpu_who = who;            /* 读到了！记录芯片 ID 供判断 */

    /* 复位 */
    (void)mpu_write8(MPU_REG_PWR_MGMT_1, 0x80u);
    HAL_Delay(100);

    /* 唤醒 + 时钟源 PLL(X gyro) */
    if (mpu_write8(MPU_REG_PWR_MGMT_1, 0x01u) != MPU_OK) { return MPU_ERR; }
    HAL_Delay(10);

    (void)mpu_write8(MPU_REG_SMPLRT_DIV, 0x04u);   /* 200Hz */
    (void)mpu_write8(MPU_REG_CONFIG, 0x04u);       /* DLPF 20Hz */
    (void)mpu_write8(MPU_REG_GYRO_CONFIG, 0x08u);  /* ±500°/s */
    (void)mpu_write8(MPU_REG_ACCEL_CONFIG, 0x08u); /* ±4g，备用 */

    HAL_Delay(10);
    (void)mpu_read8(MPU_REG_WHO_AM_I, &dummy);

    s_bias_z   = 0.0f;
    s_yaw      = 0.0f;
    s_gx = s_gy = s_gz = 0.0f;
    s_rate_z   = 0.0f;
    s_last_ms  = HAL_GetTick();
    s_ready    = 1u;
    g_mpu_ok   = 1u;
    g_mpu_bias = 0.0f;

    return MPU_OK;
}

/* 扫描 0x08~0x77，看总线上到底有什么器件。
 * 用途：g_mpu_ok = 0 且 g_mpu_who = 0 时，扫一下就知道是"没器件"还是"地址不对"。 */
void Mpu_ScanBus(void)
{
    uint16_t a;

    g_mpu_scan_cnt   = 0u;
    g_mpu_scan_first = 0u;

    for (a = 0x08u; a <= 0x77u; a++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2u, 5u) == HAL_OK)
        {
            if (g_mpu_scan_cnt == 0u) { g_mpu_scan_first = (uint8_t)a; }
            g_mpu_scan_cnt++;
        }
    }
}

void Mpu_Calibrate(void)
{
    float    sum = 0.0f;
    float    gx, gy, gz;
    uint16_t i;

    if (s_ready == 0u) { return; }

    for (i = 0u; i < 200u; i++)
    {
        if (mpu_read_gyro_raw(&gx, &gy, &gz) == MPU_OK) { sum += gz; }
        HAL_Delay(5);
    }

    s_bias_z   = sum / 200.0f;
    s_yaw      = 0.0f;
    s_last_ms  = HAL_GetTick();
    g_mpu_bias = s_bias_z;
    g_mpu_yaw  = 0.0f;
}

void Mpu_Update(void)
{
    float    gx, gy, gz;
    float    dt;
    uint32_t now;

    if (s_ready == 0u) { return; }

    if (mpu_read_gyro_raw(&gx, &gy, &gz) != MPU_OK)
    {
        return;             /* 读失败这一拍不积分，避免脏数据 */
    }

    s_gx = gx;
    s_gy = gy;
    s_gz = gz;

    s_rate_z = gz - s_bias_z;

    now = HAL_GetTick();
    dt  = (float)(now - s_last_ms) * 0.001f;
    s_last_ms = now;

    if ((dt <= 0.0f) || (dt > 0.1f)) { dt = MPU_DT_NOMINAL; }

    s_yaw += s_rate_z * dt;

    while (s_yaw >  180.0f) { s_yaw -= 360.0f; }
    while (s_yaw < -180.0f) { s_yaw += 360.0f; }

    g_mpu_gx   = s_gx;
    g_mpu_gy   = s_gy;
    g_mpu_gz   = s_gz;
    g_mpu_rate = s_rate_z;
    g_mpu_yaw  = s_yaw;
    g_mpu_bias = s_bias_z;
}

void Mpu_ResetYaw(void)
{
    s_yaw     = 0.0f;
    s_last_ms = HAL_GetTick();
    g_mpu_yaw = 0.0f;
}

void Mpu_NoiseSweep(uint16_t n)
{
    float    gx, gy, gz;
    float    sum = 0.0f;
    float    mn  =  1.0e9f;
    float    mx  = -1.0e9f;
    uint16_t i;
    uint16_t ok = 0u;

    if (s_ready == 0u) { return; }
    if (n == 0u) { n = 1u; }

    for (i = 0u; i < n; i++)
    {
        if (mpu_read_gyro_raw(&gx, &gy, &gz) == MPU_OK)
        {
            sum += gz;
            if (gz < mn) { mn = gz; }
            if (gz > mx) { mx = gz; }
            ok++;
        }
        HAL_Delay(5);
    }

    if (ok > 0u)
    {
        g_mpu_mean = sum / (float)ok;
        g_mpu_min  = mn;
        g_mpu_max  = mx;
    }
}

float   Mpu_GetYaw(void)     { return s_yaw;     }
float   Mpu_GetYawRate(void) { return s_rate_z;  }
float   Mpu_GetBiasZ(void)   { return s_bias_z;  }
float   Mpu_GetGX(void)      { return s_gx;      }
float   Mpu_GetGY(void)      { return s_gy;      }
float   Mpu_GetGZ(void)      { return s_gz;      }
uint8_t Mpu_IsReady(void)    { return s_ready;   }
