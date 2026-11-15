/**
  ******************************************************************************
  * @file    board_test.c
  * @brief   电赛测试板2（F407版）板级自检
  *
  * 硬件（按扩展板原理图 v1.0）：
  *   PWMA=PE9(TIM1_CH1)  AIN1=PE2  AIN2=PE3
  *   PWMB=PE11(TIM1_CH2) BIN1=PE4  BIN2=PE5
  *   左编码器 A/B = PA0/PB3 (TIM2)     右编码器 A/B = PA6/PA7 (TIM3)
  *   灰度 AD0/AD1/AD2/OUT = PD0/PD1/PD2/PD3
  *   OLED  SCL/SDA = PB10/PB11 (I2C2)  MPU6050 SCL/SDA/INT = PB6/PB7/PD7 (I2C1)
  *   蓝牙  TX/RX/STATE = PA10/PA9/PA8 (USART1)
  *   调试串口 TX/RX = PA2/PA3 (USART2)   按键 = PC0/PC1/PC2（上拉，按下为 0）
  *
  * 用法：
  *   1) 把 board_test.c / board_test.h 加进 MDK 工程；
  *   2) main.c:  #include "board_test.h"
  *              在 while (1) 之前调用 BoardTest_Run();
  *   3) USB-TTL 接 U19（PA2=TX, PA3=RX, GND），115200 8N1，串口助手发单字母。
  *
  * 注意：跑电机测试前必须把轮子架空！
  ******************************************************************************
  */
#include "board_test.h"
#include "main.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ============================ 句柄 ============================ */
static UART_HandleTypeDef btest_hu1;   /* USART1 蓝牙 */
static UART_HandleTypeDef btest_hu2;   /* USART2 调试 */

static I2C_HandleTypeDef  btest_hi2c1; /* PB6/PB7  MPU6050 */
static I2C_HandleTypeDef  btest_hi2c2; /* PB10/PB11 OLED  */

static TIM_HandleTypeDef  btest_htim1; /* PWM 电机 */
static TIM_HandleTypeDef  btest_htim2; /* 编码器 左 */
static TIM_HandleTypeDef  btest_htim3; /* 编码器 右 */

static ADC_HandleTypeDef  btest_hadc1; /* 内部 VREFINT 反算 VDD */

static char btest_tx[192];

/* ============================ 小工具 ============================ */

/* 控制台输出：同时打到 USART2（调试口）和 USART1（蓝牙），两个口都能看 */
static void con(const char *fmt, ...)
{
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(btest_tx, sizeof(btest_tx), fmt, ap);
  va_end(ap);

  if (n > 0)
  {
    if (n > (int)sizeof(btest_tx) - 1) n = (int)sizeof(btest_tx) - 1;
    HAL_UART_Transmit(&btest_hu2, (uint8_t *)btest_tx, (uint16_t)n, 300);
    HAL_UART_Transmit(&btest_hu1, (uint8_t *)btest_tx, (uint16_t)n, 300);
  }
}

static int con_getch(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < timeout_ms)
  {
    /* 两个串口谁先来就用谁 */
    if (__HAL_UART_GET_FLAG(&btest_hu2, UART_FLAG_RXNE))
      return (int)(btest_hu2.Instance->DR & 0xFFu);
    if (__HAL_UART_GET_FLAG(&btest_hu1, UART_FLAG_RXNE))
      return (int)(btest_hu1.Instance->DR & 0xFFu);
  }
  return -1;
}

/* 4051 通道切换后的建立时间（几十微秒，任何主频下都够） */
static void settle(void)
{
  volatile int i;
  for (i = 0; i < 40; i++) { __NOP(); }
}

/* ============================ 初始化 ============================ */

static void btest_uart_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  /* PA9=TX PA10=RX (AF7) */
  g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &g);

  /* PA2=TX PA3=RX (AF7) */
  g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &g);

  btest_hu1.Instance          = USART1;
  btest_hu1.Init.BaudRate     = 115200;
  btest_hu1.Init.WordLength   = UART_WORDLENGTH_8B;
  btest_hu1.Init.StopBits     = UART_STOPBITS_1;
  btest_hu1.Init.Parity       = UART_PARITY_NONE;
  btest_hu1.Init.Mode         = UART_MODE_TX_RX;
  btest_hu1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  btest_hu1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&btest_hu1) != HAL_OK) { Error_Handler(); }

  btest_hu2.Instance          = USART2;
  btest_hu2.Init.BaudRate     = 115200;
  btest_hu2.Init.WordLength   = UART_WORDLENGTH_8B;
  btest_hu2.Init.StopBits     = UART_STOPBITS_1;
  btest_hu2.Init.Parity       = UART_PARITY_NONE;
  btest_hu2.Init.Mode         = UART_MODE_TX_RX;
  btest_hu2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  btest_hu2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&btest_hu2) != HAL_OK) { Error_Handler(); }
}

static void btest_i2c_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  /* PB6/PB7 = I2C1 (AF4) */
  g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode      = GPIO_MODE_AF_OD;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &g);

  /* PB10/PB11 = I2C2 (AF4) */
  g.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
  g.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &g);

  btest_hi2c1.Instance             = I2C1;
  btest_hi2c1.Init.ClockSpeed      = 100000;
  btest_hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  btest_hi2c1.Init.OwnAddress1     = 0;
  btest_hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  btest_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  btest_hi2c1.Init.OwnAddress2     = 0;
  btest_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  btest_hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&btest_hi2c1) != HAL_OK) { Error_Handler(); }

  btest_hi2c2 = btest_hi2c1;
  btest_hi2c2.Instance = I2C2;
  if (HAL_I2C_Init(&btest_hi2c2) != HAL_OK) { Error_Handler(); }
}

/* 电机：PE2..PE5 方向，PE9/PE11 PWM (TIM1 CH1/CH2, 20kHz) */
static void btest_motor_init(void)
{
  GPIO_InitTypeDef  g  = {0};
  TIM_OC_InitTypeDef oc = {0};
  uint32_t pclk2, tclk, psc;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* 方向脚先拉低，保证上电不转 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
  g.Pin   = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &g);

  /* PWM 脚 */
  g.Pin       = GPIO_PIN_9 | GPIO_PIN_11;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &g);

  /* TIM1 时钟：若 APB2 分频 != 1，定时器时钟 = 2×PCLK2 */
  pclk2 = HAL_RCC_GetPCLK2Freq();
  tclk  = (pclk2 == HAL_RCC_GetHCLKFreq()) ? pclk2 : (pclk2 * 2u);
  psc   = tclk / 2000000u;          /* 2MHz 计数时钟 */
  if (psc == 0u) psc = 1u;

  btest_htim1.Instance               = TIM1;
  btest_htim1.Init.Prescaler         = psc - 1u;
  btest_htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  btest_htim1.Init.Period            = 99;          /* 2MHz/100 = 20kHz */
  btest_htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  btest_htim1.Init.RepetitionCounter = 0;
  btest_htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&btest_htim1) != HAL_OK) { Error_Handler(); }

  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&btest_htim1, &oc, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&btest_htim1, &oc, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  HAL_TIM_PWM_Start(&btest_htim1, TIM_CHANNEL_1);   /* 内部会开 MOE */
  HAL_TIM_PWM_Start(&btest_htim1, TIM_CHANNEL_2);
}

/* 编码器：TIM2(PA0/PB3, 32bit)  TIM3(PA6/PA7, 16bit) */
static void btest_encoder_init(void)
{
  GPIO_InitTypeDef      g = {0};
  TIM_Encoder_InitTypeDef s = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  g.Pin       = GPIO_PIN_0;                 /* PA0  TIM2_CH1 */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &g);

  g.Pin       = GPIO_PIN_3;                 /* PB3  TIM2_CH2 */
  HAL_GPIO_Init(GPIOB, &g);

  g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;    /* PA6/PA7 TIM3_CH1/CH2 */
  g.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &g);

  s.EncoderMode  = TIM_ENCODERMODE_TI12;
  s.IC1Polarity  = TIM_ICPOLARITY_RISING;
  s.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  s.IC1Prescaler = TIM_ICPSC_DIV1;
  s.IC1Filter    = 10;
  s.IC2Polarity  = TIM_ICPOLARITY_RISING;
  s.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  s.IC2Prescaler = TIM_ICPSC_DIV1;
  s.IC2Filter    = 10;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  btest_htim2.Instance               = TIM2;
  btest_htim2.Init.Prescaler         = 0;
  btest_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  btest_htim2.Init.Period            = 0xFFFFFFFFu;
  btest_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  btest_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&btest_htim2, &s) != HAL_OK) { Error_Handler(); }

  btest_htim3.Instance               = TIM3;
  btest_htim3.Init.Prescaler         = 0;
  btest_htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  btest_htim3.Init.Period            = 0xFFFFu;
  btest_htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  btest_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&btest_htim3, &s) != HAL_OK) { Error_Handler(); }

  __HAL_TIM_SET_COUNTER(&btest_htim2, 0);
  __HAL_TIM_SET_COUNTER(&btest_htim3, 0);
  HAL_TIM_Encoder_Start(&btest_htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&btest_htim3, TIM_CHANNEL_ALL);
}

/* 数字输入：按键、灰度 OUT、MPU INT、蓝牙 STATE；灰度地址输出 */
static void btest_gpio_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* PC0/PC1/PC2 按键，上拉 */
  g.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &g);

  /* PD3 灰度 OUT，PD7 MPU INT */
  g.Pin  = GPIO_PIN_3 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &g);

  /* PD0/PD1/PD2 灰度地址，输出 */
  g.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &g);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);

  /* PA8 蓝牙 STATE */
  g.Pin  = GPIO_PIN_8;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);
}

/* 用内部 VREFINT(1.21V) 反算 VDD，间接验证 3V3/VDDA 是否正常 */
static uint32_t btest_read_vdd_mv(void)
{
  ADC_ChannelConfTypeDef c = {0};
  uint32_t raw = 0;

  __HAL_RCC_ADC1_CLK_ENABLE();

  btest_hadc1.Instance                      = ADC1;
  btest_hadc1.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  btest_hadc1.Init.Resolution               = ADC_RESOLUTION_12B;
  btest_hadc1.Init.ScanConvMode             = DISABLE;
  btest_hadc1.Init.ContinuousConvMode       = DISABLE;
  btest_hadc1.Init.DiscontinuousConvMode    = DISABLE;
  btest_hadc1.Init.NbrOfDiscConversion      = 0;
  btest_hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  btest_hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  btest_hadc1.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  btest_hadc1.Init.NbrOfConversion          = 1;
  btest_hadc1.Init.DMAContinuousRequests    = DISABLE;
  btest_hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&btest_hadc1) != HAL_OK) { return 0; }

  c.Channel      = ADC_CHANNEL_VREFINT;
  c.Rank         = 1;
  c.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  c.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&btest_hadc1, &c) != HAL_OK) { return 0; }

  if (HAL_ADC_Start(&btest_hadc1) != HAL_OK) { return 0; }
  if (HAL_ADC_PollForConversion(&btest_hadc1, 20) == HAL_OK)
  {
    raw = HAL_ADC_GetValue(&btest_hadc1);
  }
  HAL_ADC_Stop(&btest_hadc1);

  if (raw == 0u) return 0;
  return (uint32_t)((1210ull * 4095ull) / raw);
}

/* ============================ 各模块测试 ============================ */

static void test_vdd(void)
{
  uint32_t mv = btest_read_vdd_mv();

  con("\r\n[VDD] VREFINT 反算 = %lu mV  (正常 3250~3350)\r\n", (unsigned long)mv);
  if (mv == 0u)
    con("      读取失败：ADC/VDDA 异常，或 VREF+ 被短到地\r\n");
}

static int i2c_scan(I2C_HandleTypeDef *h, uint8_t *found, int maxn)
{
  uint8_t a;
  int n = 0;

  for (a = 0x08; a < 0x78; a++)
  {
    if (HAL_I2C_IsDeviceReady(h, (uint16_t)(a << 1), 2, 3) == HAL_OK)
    {
      if (n < maxn) found[n] = a;
      n++;
    }
  }
  return n;
}

static void test_i2c(void)
{
  uint8_t f[16];
  int n, i;

  n = i2c_scan(&btest_hi2c1, f, 16);
  con("\r\n[I2C1] PB6/PB7 找到 %d 个器件:", n);
  for (i = 0; i < n && i < 16; i++) con(" 0x%02X", f[i]);
  con("   (期望 0x68 MPU6050)\r\n");

  n = i2c_scan(&btest_hi2c2, f, 16);
  con("[I2C2] PB10/PB11 找到 %d 个器件:", n);
  for (i = 0; i < n && i < 16; i++) con(" 0x%02X", f[i]);
  con("   (期望 0x3C OLED)\r\n");
}

static void test_mpu(void)
{
  uint8_t who = 0, buf[6];

  if (HAL_I2C_Mem_Read(&btest_hi2c1, (0x68 << 1), 0x75, I2C_MEMADD_SIZE_8BIT, &who, 1, 20) != HAL_OK)
  {
    con("\r\n[MPU] 读 WHO_AM_I 失败：I2C 没通（查 PB6/PB7、上拉、供电）\r\n");
    return;
  }
  con("\r\n[MPU] WHO_AM_I = 0x%02X (期望 0x68)\r\n", who);

  /* 唤醒 + 量程 */
  {
    uint8_t v = 0x00;
    HAL_I2C_Mem_Write(&btest_hi2c1, (0x68 << 1), 0x6B, I2C_MEMADD_SIZE_8BIT, &v, 1, 20); /* PWR_MGMT_1 */
    v = 0x00;
    HAL_I2C_Mem_Write(&btest_hi2c1, (0x68 << 1), 0x1C, I2C_MEMADD_SIZE_8BIT, &v, 1, 20); /* ACCEL ±2g */
  }
  HAL_Delay(50);

  if (HAL_I2C_Mem_Read(&btest_hi2c1, (0x68 << 1), 0x3B, I2C_MEMADD_SIZE_8BIT, buf, 6, 20) == HAL_OK)
  {
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    con("      ACC = %d, %d, %d   (静止时某一个轴≈16384)\r\n", ax, ay, az);
  }
}

static void test_buttons(void)
{
  uint32_t t0 = HAL_GetTick();

  con("\r\n[BUTTON] 5 秒内随便按几个键...\r\n");
  while ((HAL_GetTick() - t0) < 5000u)
  {
    con("      PC0=%d PC1=%d PC2=%d\r\n",
        (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0),
        (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1),
        (int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2));
    HAL_Delay(300);
  }
}

static uint8_t track_read(void)
{
  uint8_t mask = 0;
  uint8_t i;
  GPIO_PinState s;

  for (i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, (i & 0x01u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (i & 0x02u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, (i & 0x04u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    settle();
    s = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);
    if (s == GPIO_PIN_SET) mask |= (uint8_t)(1u << i);
  }
  return mask;
}

static void test_track(void)
{
  uint32_t t0 = HAL_GetTick();
  uint8_t m;

  con("\r\n[TRACK] 10 秒，把探头在黑线/白纸上移动\r\n");
  while ((HAL_GetTick() - t0) < 10000u)
  {
    m = track_read();
    con("      CH8..CH1 = %d%d%d%d %d%d%d%d   0x%02X\r\n",
        (m >> 7) & 1, (m >> 6) & 1, (m >> 5) & 1, (m >> 4) & 1,
        (m >> 3) & 1, (m >> 2) & 1, (m >> 1) & 1, m & 1, m);
    HAL_Delay(150);
  }
}

static void test_encoder(void)
{
  uint32_t t0 = HAL_GetTick();
  int32_t  cnt2 = 0, cnt3 = 0;
  int16_t  last3;

  __HAL_TIM_SET_COUNTER(&btest_htim2, 0);
  __HAL_TIM_SET_COUNTER(&btest_htim3, 0);
  last3 = (int16_t)__HAL_TIM_GET_COUNTER(&btest_htim3);

  con("\r\n[ENCODER] 10 秒，用手转两个轮子\r\n");
  while ((HAL_GetTick() - t0) < 10000u)
  {
    int32_t  c2 = (int32_t)__HAL_TIM_GET_COUNTER(&btest_htim2);
    int16_t  now3 = (int16_t)__HAL_TIM_GET_COUNTER(&btest_htim3);
    int16_t  d3 = (int16_t)(now3 - last3);

    cnt2 = c2;
    cnt3 += d3;
    last3 = now3;

    con("      TIM2(左)=%ld  TIM3(右)=%ld  (PA8 STATE=%d)\r\n",
        (long)cnt2, (long)cnt3,
        (int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8));
    HAL_Delay(200);
  }
}

/* 左轮：l>0 正转，l<0 反转；右轮同理。-100..100 */
static void motor_set(int l, int r)
{
  uint32_t dl, dr;

  if (l > 0)      { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); dl = (uint32_t)l; }
  else if (l < 0) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   dl = (uint32_t)(-l); }
  else            { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); dl = 0; }

  if (r > 0)      { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); dr = (uint32_t)r; }
  else if (r < 0) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);   dr = (uint32_t)(-r); }
  else            { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); dr = 0; }

  __HAL_TIM_SET_COMPARE(&btest_htim1, TIM_CHANNEL_1, dl);
  __HAL_TIM_SET_COMPARE(&btest_htim1, TIM_CHANNEL_2, dr);
}

static void motor_brake(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&btest_htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&btest_htim1, TIM_CHANNEL_2, 0);
}

static void test_motor(void)
{
  int d;

  con("\r\n[MOTOR] 轮子架空！3 秒后开始：正转 30%% -> 反转 30%% -> 停\r\n");
  HAL_Delay(3000);

  con("      正转 30%%\r\n");
  for (d = 0; d <= 30; d += 5) { motor_set(d, d); HAL_Delay(100); }
  HAL_Delay(1500);

  con("      反转 30%%\r\n");
  for (d = 0; d <= 30; d += 5) { motor_set(-d, -d); HAL_Delay(100); }
  HAL_Delay(1500);

  con("      刹车\r\n");
  motor_brake();
  HAL_Delay(800);
  motor_set(0, 0);
  con("      停\r\n");
}

static void test_menu(void)
{
  con("\r\n");
  con("==============================================\r\n");
  con("  电赛测试板2 (F407) 自检\r\n");
  con("  v: 读VDD   i: I2C扫描   m: MPU6050\r\n");
  con("  b: 按键    t: 灰度     e: 编码器\r\n");
  con("  p: 电机PWM(架空)  s: 电机单步   h: 帮助\r\n");
  con("==============================================\r\n");
}

/* ============================ 入口 ============================ */

void BoardTest_Run(void)
{
  btest_uart_init();
  btest_i2c_init();
  btest_motor_init();
  btest_encoder_init();
  btest_gpio_init();

  test_menu();

  for (;;)
  {
    int c = con_getch(500);
    if (c < 0) continue;

    /* 回显，方便确认按键 */
    con("\r\n>>> %c\r\n", (char)c);

    switch (c)
    {
      case 'v': case 'V': test_vdd();      break;
      case 'i': case 'I': test_i2c();      break;
      case 'm': case 'M': test_mpu();      break;
      case 'b': case 'B': test_buttons();  break;
      case 't': case 'T': test_track();    break;
      case 'e': case 'E': test_encoder();  break;
      case 'p': case 'P': test_motor();    break;
      case 's': case 'S': motor_set(25, 25);  HAL_Delay(1000); motor_set(-25, -25); HAL_Delay(1000); motor_set(0, 0);  con("      单步完成\r\n"); break;
      case 'h': case 'H': test_menu();     break;
      default:  con("      未知命令，按 h 看帮助\r\n"); break;
    }
  }
}
