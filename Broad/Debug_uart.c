/**
  ******************************************************************************
  * @file    Debug_uart.c
  * @brief   USART2 上的 CSV 调试输出（SerialPlot 用），非阻塞发送
  *
  *  设计要点：
  *    1) 不用 printf、不用 MicroLIB、不依赖 stdio
  *    2) 不用 HAL_UART_Transmit（那是阻塞的，一行 60 字节要 5ms，
  *       在 10ms 控制周期下会把节拍拖歪）
  *    3) 改成"格式化进缓冲区 + 主循环逐字节喂 TXE"，全程不阻塞
  *
  *  CPU 占用：20Hz × 60 字节 ≈ 1200 B/s，占 115200 带宽的 10%，
  *            而每次喂字节只是一个寄存器写，不消耗时间。
  ******************************************************************************
  */

#include "Debug_uart.h"
#include "usart.h"
#include "Trace.h"
#include "Motor_advance.h"
#include "Mpu6050.h"

/* 打印周期（ms），50 = 20Hz */
#define DBG_PERIOD_MS      50

/* 1 = 支持串口下发目标速度（调速度环用） */
#define DBG_CMD_ENABLE     1

#define DBG_BUF_SIZE       96

static uint16_t s_period_ms = DBG_PERIOD_MS;
static uint32_t s_last_ms   = 0;
static uint8_t  s_enable    = 1;

/* 发送缓冲：s_pos == s_len 表示空闲，可以格式化下一行 */
static char     s_buf[DBG_BUF_SIZE];
static uint16_t s_len = 0;
static uint16_t s_pos = 0;

/* 手动速度模式 */
static uint8_t  s_manual = 0;

#if (DBG_CMD_ENABLE != 0)
static int32_t  s_rx_int  = 0;      /* 整数部分 */
static int32_t  s_rx_frac = 0;      /* 小数部分 */
static uint8_t  s_rx_ndec = 0u;     /* 小数位数 */
static uint8_t  s_rx_neg  = 0u;
static uint8_t  s_rx_dot  = 0u;
static uint8_t  s_rx_has  = 0u;
static char     s_rx_key  = 0;      /* 命令字母，0 = 纯数字（手动速度） */
static uint32_t s_rx_ms   = 0;
#endif

/* ------------------------------ 串口初始化 ------------------------------ */

static void dbg_uart_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;          /* RX 上拉，没接串口时不收噪声 */
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    /* BRR = USARTDIV×16 = PCLK/baud，USART2 在 APB1（42MHz） */
    USART2->BRR = (HAL_RCC_GetPCLK1Freq() + (115200u / 2u)) / 115200u;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* ------------------------------ 数字转 ASCII ------------------------------ */

static uint16_t put_i32(char *p, int32_t v)
{
    char     tmp[12];
    uint16_t k = 0;
    uint16_t n = 0;
    uint32_t u;

    if (v < 0)
    {
        p[n++] = '-';
        u = (uint32_t)(-(v + 1)) + 1u;      /* 防 INT32_MIN 取反溢出 */
    }
    else
    {
        u = (uint32_t)v;
    }

    if (u == 0u)
    {
        p[n++] = '0';
        return n;
    }

    while (u > 0u)
    {
        tmp[k++] = (char)('0' + (u % 10u));
        u /= 10u;
    }
    while (k > 0u)
    {
        p[n++] = tmp[--k];
    }
    return n;
}

/* 四舍五入取整，避免小数值全被截断成 0 */
static int32_t round_i(float v)
{
    return (v >= 0.0f) ? (int32_t)(v + 0.5f) : (int32_t)(v - 0.5f);
}

/* ------------------------------ 组帧 ------------------------------ */

static void dbg_format_line(void)
{
    uint16_t n = 0;
    uint8_t  mode;

    if (s_manual != 0u)                     { mode = 5u; }
    else if (Trace_GetState() != TRACE_RUN) { mode = 4u; }
    else if (g_trace_blind != 0u)           { mode = 1u; }
    else if (g_trace_ft    != 0u)           { mode = 2u; }
    else if (g_trace_fin   != 0u)           { mode = 3u; }
    else                                    { mode = 0u; }

    n += put_i32(&s_buf[n], (int32_t)MotorAdvance_GetTargetL()); s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)MotorAdvance_GetSpeedL());  s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)MotorAdvance_GetDutyL());   s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)MotorAdvance_GetTargetR()); s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)MotorAdvance_GetSpeedR());  s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)MotorAdvance_GetDutyR());   s_buf[n++] = ',';
    n += put_i32(&s_buf[n], round_i(g_trace_err * 10.0f));       s_buf[n++] = ',';
    n += put_i32(&s_buf[n], round_i(g_trace_steer));             s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)g_trace_mask);              s_buf[n++] = ',';
    n += put_i32(&s_buf[n], round_i(Mpu_GetYaw() * 10.0f));      s_buf[n++] = ',';
    n += put_i32(&s_buf[n], round_i(Mpu_GetYawRate() * 10.0f));  s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)mode);                      s_buf[n++] = ',';
    n += put_i32(&s_buf[n], round_i(g_trace_ye * 10.0f));        /* 航向误差 ×10 */
    s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)g_trace_lev);               /* 第几次丢线 */
    s_buf[n++] = ',';
    n += put_i32(&s_buf[n], (int32_t)g_trace_dist);              /* 累计里程 mm */
    s_buf[n++] = '\r';
    s_buf[n++] = '\n';

    s_len = n;
    s_pos = 0;
}

/* 非阻塞泵：TXE 空就喂一个字节，喂完立即返回 */
static void dbg_pump(void)
{
    if (s_pos >= s_len) { return; }

    if ((USART2->SR & USART_SR_TXE) != 0u)
    {
        USART2->DR = (uint16_t)(uint8_t)s_buf[s_pos];
        s_pos++;
    }
}

/* ------------------------------ 下行命令 ------------------------------ */

#if (DBG_CMD_ENABLE != 0)

/* 数字停 120ms 没新字节就认为输入结束（不依赖串口助手发不发换行） */
#define DBG_CMD_IDLE_MS    120

/* 把收到的整数+小数拼成 float */
static float rx_value(void)
{
    float v = (float)s_rx_int;

    if (s_rx_ndec == 1u) { v += (float)s_rx_frac * 0.1f; }
    else if (s_rx_ndec == 2u) { v += (float)s_rx_frac * 0.01f; }

    return (s_rx_neg != 0u) ? -v : v;
}

static void rx_clear(void)
{
    s_rx_int  = 0;
    s_rx_frac = 0;
    s_rx_ndec = 0u;
    s_rx_neg  = 0u;
    s_rx_dot  = 0u;
    s_rx_has  = 0u;
    s_rx_key  = 0;
}

static void dbg_cmd_apply(void)
{
    float v;

    if (s_rx_has == 0u) { rx_clear(); return; }

    v = rx_value();

    switch (s_rx_key)
    {
        case 'p': g_kp_trace      = v;  break;   /* 循迹 KP */
        case 'd': g_kd_trace      = v;  break;   /* 循迹 KD */
        case 'b': g_base_mm_s     = v;  break;   /* 基础车速 */
        case 's': g_search_steer  = v;  break;   /* 找线差速 */
        case 'l': g_blind_speed   = v;  break;   /* 盲走速度 */
        case 'y': g_kp_yaw        = v;  break;   /* 盲走航向增益 */
        case 'g': g_blind_offset  = v;  break;   /* 盲走目标偏移角 */
        case 'm': g_steer_limit   = v;  break;   /* 航向修正限幅 */
        case 't': g_island_turn_speed = v; break; /* 环岛转向速度 */
        case 'a': g_island_turn_deg   = v; break; /* 环岛转向角度 */
        case 'f': g_lpf_alpha         = v; break; /* 误差低通，1 = 关闭 */
        case 'w': g_island_ramp_mm    = v; break; /* 环岛转向后速度渐变距离 */
        case 'n': g_slow_start_mm     = v; break; /* 盲走前减速起始里程 */
        case 'k': g_slow_speed        = v; break; /* 盲走前慢速 */
        case 'q': g_loss_debounce_ms  = (uint16_t)v; break; /* 丢线消抖 ms */
        case 'o': g_corner6_gap_mm    = v; break; /* 第6→7个弯最小间距 */
        case 'v': g_kd_yaw_rate       = v; break; /* 航向速率阻尼 */

        case 0:                                   /* 纯数字 = 手动速度 */
            s_manual = 1u;
            MotorAdvance_SetTarget((int16_t)v, (int16_t)v);
            break;

        default: break;
    }

    rx_clear();
}

static void dbg_poll_rx(void)
{
    while ((USART2->SR & USART_SR_RXNE) != 0u)
    {
        char c = (char)(USART2->DR & 0xFFu);

        if ((c >= '0') && (c <= '9'))
        {
            if (s_rx_dot == 0u)
            {
                if (s_rx_int < 100000) { s_rx_int = (s_rx_int * 10) + (int32_t)(c - '0'); }
            }
            else if (s_rx_ndec < 2u)
            {
                s_rx_frac = (s_rx_frac * 10) + (int32_t)(c - '0');
                s_rx_ndec++;
            }
            s_rx_has = 1u;
            s_rx_ms  = HAL_GetTick();
        }
        else if (c == '.')
        {
            s_rx_dot = 1u;
            s_rx_has = 1u;
            s_rx_ms  = HAL_GetTick();
        }
        else if (c == '-')
        {
            s_rx_neg = 1u;
            s_rx_has = 1u;
            s_rx_ms  = HAL_GetTick();
        }
        else if ((c >= 'a') && (c <= 'z'))
        {
            if (c == 'c')
            {
                /* 退出手动模式，恢复循迹 */
                s_manual = 0u;
                rx_clear();
                Trace_Start();
            }
            else if (s_rx_has != 0u)
            {
                dbg_cmd_apply();        /* 连续两条命令，先结算上一条 */
                s_rx_key = c;
                s_rx_has = 0u;
                s_rx_ms  = HAL_GetTick();
            }
            else
            {
                s_rx_key = c;
                s_rx_has = 1u;
                s_rx_ms  = HAL_GetTick();
            }
        }
        else if ((c == '\r') || (c == '\n'))
        {
            dbg_cmd_apply();
        }
    }
}

static void dbg_cmd_tick(void)
{
    if ((s_rx_has != 0u) && ((HAL_GetTick() - s_rx_ms) >= DBG_CMD_IDLE_MS))
    {
        dbg_cmd_apply();
    }
}

#endif /* DBG_CMD_ENABLE */

/* ------------------------------ 对外接口 ------------------------------ */

void Dbg_Init(void)
{
    static const char hdr[] =
        "#tgtL,spdL,dtyL,tgtR,spdR,dtyR,errx10,steer,mask,yawx10,ratex10,mode,yex10,lev,dist\r\n";

    dbg_uart_init();

    s_last_ms = HAL_GetTick();
    s_enable  = 1u;
    s_len     = (uint16_t)(sizeof(hdr) - 1u);
    s_pos     = 0u;

    /* 表头直接塞进缓冲区，由 dbg_pump 慢慢发 */
    {
        uint16_t i;
        for (i = 0u; i < s_len; i++) { s_buf[i] = hdr[i]; }
    }
}

void Dbg_Task(void)
{
    uint32_t now;

    dbg_pump();                 /* 先把上一行发完 */

    if (s_enable == 0u) { return; }

#if (DBG_CMD_ENABLE != 0)
    dbg_poll_rx();
    dbg_cmd_tick();
#endif

    if (s_pos < s_len) { return; }   /* 上一行还没发完，不覆盖 */

    now = HAL_GetTick();
    if ((now - s_last_ms) < (uint32_t)s_period_ms) { return; }
    s_last_ms = now;

    dbg_format_line();
    dbg_pump();
}

void Dbg_Enable(uint8_t on)
{
    s_enable = (on != 0u) ? 1u : 0u;
}

uint8_t Dbg_IsManual(void)
{
    return s_manual;
}
