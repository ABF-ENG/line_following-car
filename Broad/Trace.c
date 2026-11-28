/**
  ******************************************************************************
  * @file    Trace.c
  * @brief   八路灰度循迹决策：误差计算 + 特殊要素识别 + 目标速度分配
  *
  *  数据流： Gray_ReadMask() → 本文件 → MotorAdvance_SetTarget()
  *
  *  极性（实测确认）：mask 某位 = 1 表示该探头压到黑线
  *
  *  处理分支（按优先级）：
  *     1. 全黑 + 已见过正常线型 + 持续够久   → 终点，停车
  *     2. 全黑（短暂）                        → 十字 / 圆环，直行通过
  *     3. 全白                                → 丢线，朝最后看到的方向找，超时停车
  *     4. 正常 1~4 路                         → 加权重心 + PD 差速
  ******************************************************************************
  */

#include "Trace.h"
#include "Gray.h"
#include "Motor_base.h"
#include "Motor_advance.h"
#include "Mpu6050.h"

/* ==================== 盲走前减速 ====================
 * 第 5 个直角之后开始计里程，走够 SLOW_START_MM 毫米就降到 SLOW_SPEED_MM_S，
 * 一直保持到盲走段走完（s_blind_done 置 1）再恢复 BASE。
 * 目的：盲走前的最后两个直角和断口本身都慢一点，冲出去的风险小很多。 */
#define SLOW_AFTER_CORNER   5u          /* 第几个直角之后开始计里程 */
#define SLOW_START_MM_DEF   1400.0f     /* 再走多少毫米开始减速 */
#define SLOW_SPEED_MM_S_DEF 300.0f      /* 减速后的速度 */

/* ==================== 丢线计数消抖 ====================
 * 一次直角弯里，线可能反复"丢了又看见"好几次，把丢线次数刷得飞快。
 * 所以规定：距上次计数不足 LOSS_DEBOUNCE_MS 的丢线一律忽略、不计次。
 * 500ms = 400mm/s 下走 200mm，比两个直角弯之间的间距小，不会误吞。
 * 串口 "q500" 可改。 */
#define LOSS_DEBOUNCE_MS_DEF  500u
uint16_t g_loss_debounce_ms = LOSS_DEBOUNCE_MS_DEF;

/* 第 6 个弯之后，必须再走这么多毫米才允许数出第 7 个弯。
 * 目的：第 6 个弯车身还没摆正时线又丢一下，会被误数成第 7 个弯。
 * 串口 "o200" 可改。 */
#define CORNER6_GAP_MM_DEF    200.0f
float g_corner6_gap_mm = CORNER6_GAP_MM_DEF;

float g_slow_start_mm = SLOW_START_MM_DEF;   /* 串口 "n1200" 可改 */
float g_slow_speed    = SLOW_SPEED_MM_S_DEF; /* 串口 "k300" 可改 */

/* ======================= 可调参数（调循迹只动这一段） =======================
 *
 * 下面这些参数是**运行时可改**的：串口发 "p35" 就能把 KP 改成 35，
 * 不用重新编译下载。代码里仍然用 KP_TRACE / KD_TRACE 这样的名字，
 * 它们实际指向 g_* 变量。
 *
 * 串口命令（SerialPlot 的 Commands 页，或任意串口助手）：
 *     p35     KP_TRACE   = 35
 *     d12.5   KD_TRACE   = 12.5      （支持小数，最多两位）
 *     b400    BASE_MM_S  = 400
 *     s340    SEARCH_STEER_MM_S = 340
 *     l500    BLIND_SPEED_MM_S  = 500
 *     y7      KP_YAW     = 7
 *     g180    BLIND_OFFSET_DEG = 180
 *     300     手动模式：左右轮都跑 300 mm/s（Trace 不再覆盖目标）
 *     c       退出手动，恢复循迹
 * 每条命令后面加回车换行；不加也行，停 120ms 自动生效。
 * ------------------------------------------------------------------------- */

/* 正常循迹的基础车速 mm/s */
#define BASE_MM_S_DEF       420.0f

/* 比例增益：每 1 个误差单位给多少 mm/s 的差速。误差范围 ±7 */
#define KP_TRACE_DEF        30.0f

/* 微分增益：作用在"每拍误差变化量"上，抑制摆动。太大对传感器跳变敏感
 * ★ 它没有除以采样周期，等效阻尼 = KD × CTRL_PERIOD_MS。
 *   周期 20ms → 10ms 时等效阻尼砍半，所以 KD 要翻倍（15 → 30）补回来。 */
#define KD_TRACE_DEF        15.0f

/* 丢线时的找线差速 mm/s */
#define SEARCH_STEER_MM_S_DEF  340.0f

/* 车轮速度限幅：内侧轮别停死（停死等于原地转，容易拖出赛道） */
#define SPEED_MIN_MM_S      60.0f
#define SPEED_MAX_MM_S     900.0f

/* 判为"宽黑线"（十字/圆环/终点）所需的最少探头数，8 路里 5 路以上算宽 */
#define WIDE_COUNT          6u

/* ---- 运行时可改的实体 ---- */
float g_base_mm_s       = BASE_MM_S_DEF;
float g_kp_trace        = KP_TRACE_DEF;
float g_kd_trace        = KD_TRACE_DEF;
float g_search_steer    = SEARCH_STEER_MM_S_DEF;

#define BASE_MM_S           (g_base_mm_s)
#define KP_TRACE            (g_kp_trace)
#define KD_TRACE            (g_kd_trace)
#define SEARCH_STEER_MM_S   (g_search_steer)

/* ============================ 起终点黑线 ============================ */

/* 盲走之后 → 环岛触发 */
#define ISLAND_WATCH_MM     600.0f   /* 从盲走出口再走多远开始盯边灯 */
#define ISLAND_EDGE_BIT     7u       /* 右边第 1 颗灯 = X8 = bit7（装反了就改成 0） */
#define ISLAND_EDGE_N       2u       /* 第几次上升沿才动作 */
#define ISLAND_TURN_DEG_DEF     30.0f    /* 强制转多少度。串口 "a20" 可改 */
#define ISLAND_TURN_SIGN    (-1.0f)  /* +1 = 逆时针(左转)，-1 = 顺时针(右转) */
#define ISLAND_TURN_SPEED_DEF   220.0f   /* 转向时的轮速 mm/s。串口 "t380" 可改
                                           * ★ 转向专用；转完会平滑升回 BASE */
#define ISLAND_TURN_TICKS   40u      /* 转向超时 40×20ms = 800ms */

/* ---- 运行时可改的环岛实体 ---- */
float g_island_turn_speed = ISLAND_TURN_SPEED_DEF;
float g_island_turn_deg   = ISLAND_TURN_DEG_DEF;

#define ISLAND_TURN_SPEED   (g_island_turn_speed)
#define ISLAND_TURN_DEG     (g_island_turn_deg)

/* 第几次压到"全黑线"算终点。这条赛道按你数的来 */
#define FINISH_CROSS_N      4u

/* 从第 N 次压线那一刻起，再直行多少 mm 然后停车 */
#define FINISH_RUN_MM       200.0f

/* 终点冲刺段的速度 mm/s */
#define  FINISH_SPEED_MM_S   350.0f

/* 全白持续多少拍放弃找线并停车。50 拍 = 1s（现在只做统计不再停车） */
#define LOST_STOP_TICKS     50u

/* 如果车"越修越偏"（往线的反方向打舵），把这里改成 1 */
#define TRACE_REVERSE       0

/* 误差一阶低通系数：1.0 = 关闭滤波；越小越平滑、越迟钝
 *   20ms 周期下 Δe 里量化跳变占比不小，KD 会把它放大成抖动。
 *   串口发 "f0.6" 可改，发 "f1" 等于关掉。 */
#define TRACE_LPF_ALPHA_DEF 0.6f
float g_lpf_alpha = TRACE_LPF_ALPHA_DEF;

/* 出环岛强制转向后，速度从 ISLAND_TURN_SPEED 平滑升回 BASE 用多少拍。
 * 防止"一恢复 400 就冲出去"。12 拍 = 240ms。0 = 不缓升。 */
#define SOFT_START_TICKS    12u

/* 环岛转向后：速度从 ISLAND_TURN_SPEED 平滑升回 BASE，走完这么多毫米才升满。
 *   220 → 400 的过渡就靠它，数值越大过渡越温和。
 *   0 = 不渐变（直接跳回 BASE）。串口 "w250" 可改。 */
#define ISLAND_RAMP_MM_DEF     1000.0f
float g_island_ramp_mm = ISLAND_RAMP_MM_DEF;
#define ISLAND_RAMP_MM         (g_island_ramp_mm)

/* ============================ 盲走（断线） ============================ */

/* 盲走速度 mm/s —— 比循迹略慢，冲过断口更可控。串口 "l500" 可改 */
#define BLIND_SPEED_MM_S_DEF   500.0f

/* 航向保持增益：每 1° 偏差给多少 mm/s 的差速。串口 "y7" 可改 */
#define KP_YAW_DEF              12.0f

/* 航向环的速率阻尼（D 项）。直接用陀螺仪角速度，不用差分，噪声小。
 * 单位：(mm/s) / (°/s)
 *   0     = 纯 P（默认）
 *   0.5~3 = 车在目标角附近来回摆时加上，压住过冲
 * 串口 "v1" 可改。 */
#define KD_YAW_RATE_DEF         0.0f

/* 航向修正的差速限幅，防止一上来猛打。串口 "m300" 可改 */
#define STEER_LIMIT_MM_S_DEF  300.0f

/* 盲走超时：150 拍 = 3s，400mm/s × 3s = 1.2m，足够过任何断口 */
#define BLIND_TIMEOUT_TICKS 150u

/* ==================== 直角弯 / 盲走：按次数写死 ====================
 * 这条赛道：盲走段之前正好 7 个直角弯，盲走段之后不再有直角弯。
 * 所以不靠"误差大小"判断，直接数丢线次数：
 *   第 1~7 次丢线  → 直角弯，朝最后看到的方向打舵找线
 *   第 8 次及以后  → 盲走（断口），陀螺仪保持航向
 * 直角弯数量变了就改这个数。 */
#define CORNER_COUNT        7u

/* 盲走的目标航向 = 发车时航向 + 这个偏移角（度）
 *   180 = 盲走段和发车段平行且方向相反（按你这条赛道）
 *   如果实测发现盲走段其实和发车段同向，改成 0
 * 进盲走那一下看 g_trace_ye：接近 0 说明偏移角对了；接近 180 说明该改成 0。 */
#define BLIND_OFFSET_DEG_DEF   180.0f

/* ---- 运行时可改的盲走实体 ---- */
float g_blind_speed     = BLIND_SPEED_MM_S_DEF;
float g_kp_yaw          = KP_YAW_DEF;
float g_kd_yaw_rate     = KD_YAW_RATE_DEF;   /* 航向速率阻尼 */
float g_steer_limit     = STEER_LIMIT_MM_S_DEF;
float g_blind_offset    = BLIND_OFFSET_DEG_DEF;

#define BLIND_SPEED_MM_S    (g_blind_speed)
#define KP_YAW              (g_kp_yaw)
#define STEER_LIMIT_MM_S    (g_steer_limit)
#define BLIND_OFFSET_DEG    (g_blind_offset)

/* 重新看到线要连续几拍才退出盲走 */
#define REACQ_TICKS         2u

/* ============================ 内部状态 ============================ */

/* 权重：X1 在最左为 -7，X8 在最右为 +7，单位 ≈ 一个探头间距 */
static const float s_weight[8] =
{
    -7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f
};

static TraceState s_state       = TRACE_IDLE;
static uint8_t    s_mask        = 0u;
static uint8_t    s_seen_track  = 0u;    /* 离开发车区后置 1，之后才允许判终点 */
static uint8_t    s_lost_cnt    = 0u;    /* 全白连续计数 */
static uint8_t    s_in_loss     = 0u;    /* 1 = 当前处于一次"丢线"中 */
static uint8_t    s_loss_ev     = 0u;    /* 丢线次数（第几次丢线） */
static uint8_t    s_blind_done  = 0u;    /* 1 = 盲走段已经走过了，之后不再进盲走 */
static uint32_t   s_loss_ms     = 0u;    /* 上一次被计数的丢线时刻 */
static uint8_t    s_loss_have   = 0u;    /* 1 = s_loss_ms 已经有效 */

/* ---- 第 6 个弯之后的最小间距门槛 ---- */
static uint8_t    s_gap_valid   = 0u;    /* 1 = s_gap_d0 已经记录 */
static float      s_gap_d0l     = 0.0f;
static float      s_gap_d0r     = 0.0f;

/* ---- 盲走前减速 ---- */
static uint8_t    s_slow_arm    = 0u;    /* 1 = 已过第 5 个直角，正在计里程 */
static uint8_t    s_slow_on     = 0u;    /* 1 = 已降到慢速，直到盲走结束 */
static float      s_slow_d0l    = 0.0f;
static float      s_slow_d0r    = 0.0f;

/* ---- 起终点黑线 ---- */
static uint8_t    s_black_edge  = 0u;    /* 上一拍是否全黑（检测上升沿） */
static uint8_t    s_cross_idx   = 0u;    /* 已经压到几次全黑线 */
static uint8_t    s_finishing   = 0u;    /* 1 = 正在终点冲刺段 */
static float      s_finish_d0l  = 0.0f;  /* 冲刺起点（左轮里程） */
static float      s_finish_d0r  = 0.0f;  /* 冲刺起点（右轮里程） */

static float      s_last_e      = 0.0f;  /* 上一次的误差（丢线/宽黑时保持） */
static float      s_e_lpf       = 0.0f;  /* 低通后的误差 */
static uint8_t    s_skip_d      = 0u;    /* 1 = 这一拍不做微分 */
static uint8_t    s_ramp_on     = 0u;    /* 1 = 正在从转向速度升回 BASE */
static float      s_ramp_d0l    = 0.0f;
static float      s_ramp_d0r    = 0.0f;

/* ---- 盲走目标：用第 3~4 弯之间那段直道的航向 ----
 * 那段直道和盲走段平行同向，所以直接拿它的航向当目标，
 * 比"发车航向 + 180°"离得近得多 —— 零偏漂移窗口从 40 秒缩到 15 秒。 */
static float      s_yaw_same    = 0.0f;  /* 那段直道录下来的航向（最近 N 个样本的平均） */
static uint8_t    s_same_ok     = 0u;    /* 1 = 已经录到了 */
static float      s_same_base   = 0.0f;  /* 第一个样本的航向，用来避开 ±180 折返 */
static uint8_t    s_same_has    = 0u;    /* 1 = base 已经取过 */

/* 只保留最近 SAME_AVG_N 个样本求平均 —— 这样用的是第 4 个弯之前
 * 最后那段"已经走稳"的航向，离盲走入口最近，也把单点噪声平均掉了。
 * 400mm/s、20ms 下 20 个样本 ≈ 0.4 秒 ≈ 160mm。 */
#define SAME_AVG_N     20u
static float      s_same_buf[SAME_AVG_N];
static uint8_t    s_same_idx    = 0u;    /* 环形缓冲写指针 */
static uint8_t    s_same_fill   = 0u;    /* 已填入的有效样本数（最多 SAME_AVG_N） */
static float      s_error       = 0.0f;
static float      s_steer       = 0.0f;
static int16_t    s_tgt_l       = 0;
static int16_t    s_tgt_r       = 0;

/* ---- 盲走状态 ---- */
static uint8_t    s_blind       = 0u;    /* 1 = 正在盲走 */
static float      s_yaw_start   = 0.0f;  /* 发车那一刻的航向（赛道的绝对参考） */
static float      s_yaw_ref     = 0.0f;  /* 盲走的目标航向 */
static uint8_t    s_blind_tick  = 0u;    /* 盲走已持续几拍 */
static uint8_t    s_reacq       = 0u;    /* 重新看到线的连续拍数 */

/* ---- 盲走之后 → 环岛触发 ---- */
static uint8_t    s_ab_on       = 0u;    /* 1 = 刚从盲走出来，正在等 60cm */
static float      s_ab_d0l      = 0.0f;
static float      s_ab_d0r      = 0.0f;
static uint8_t    s_edge_watch  = 0u;    /* 1 = 里程够了，开始盯边灯 */
static uint8_t    s_edge_last   = 0u;    /* 上一拍的边灯状态（做上升沿用） */
static uint8_t    s_edge_cnt    = 0u;    /* 边灯亮过几次 */
static uint8_t    s_ft_on       = 0u;    /* 1 = 正在强制转向 */
static float      s_ft_target   = 0.0f;  /* 转向目标航向 */
static uint8_t    s_ft_tick     = 0u;    /* 转向已持续几拍 */

/* ---- 调试镜像：Keil Watch 不能调函数，只能看变量 ---- */
volatile uint8_t g_trace_mask  = 0u;
volatile float   g_trace_err   = 0.0f;
volatile float   g_trace_steer = 0.0f;
volatile int16_t g_trace_tgtl  = 0;
volatile int16_t g_trace_tgtr  = 0;
volatile uint8_t g_trace_blind = 0u;
volatile float   g_trace_ye    = 0.0f;   /* 盲走时的航向误差，度 */
volatile uint8_t g_trace_cross = 0u;     /* 已压到几次全黑线 */
volatile uint8_t g_trace_fin   = 0u;     /* 1 = 正在终点冲刺 */
volatile uint8_t g_trace_ab    = 0u;     /* 1 = 盲走已出来，正在计 60cm */
volatile uint8_t g_trace_edge  = 0u;     /* 边灯已亮过几次 */
volatile uint8_t g_trace_ft    = 0u;     /* 1 = 正在强制转向 */
volatile uint8_t g_trace_lev   = 0u;     /* 第几次丢线（诊断用） */
volatile float   g_trace_dist  = 0.0f;   /* 累计里程 mm（诊断用） */
/* ============================== 小工具 ============================== */

static uint8_t popcount8(uint8_t v)
{
    uint8_t n = 0u;

    while (v != 0u)
    {
        n += (uint8_t)(v & 0x01u);
        v >>= 1;
    }
    return n;
}

static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float fabs_f(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 把角度归一化到 ±180 */
static float norm180(float a)
{
    while (a >  180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

/* 加权重心：mask 里为 1 的位按权重求平均 */
static float calc_error(uint8_t mask)
{
    float sum = 0.0f;
    float cnt = 0.0f;
    uint8_t i;

    for (i = 0u; i < 8u; i++)
    {
        if (((mask >> i) & 0x01u) != 0u)
        {
            sum += s_weight[i];
            cnt += 1.0f;
        }
    }

    if (cnt < 0.5f) { return s_last_e; }
    return sum / cnt;
}

/* 盲走前减速：慢速生效时把任何速度压到 g_slow_speed 以内 */
static float cap_slow(float v)
{
    if ((s_slow_on != 0u) && (v > g_slow_speed)) { return g_slow_speed; }
    return v;
}

/* 把转向量分配到左右轮，并限幅下发 */
static void apply_steer(float base, float steer)
{
    float l = base + steer;
    float r = base - steer;

    l = clamp_f(l, SPEED_MIN_MM_S, SPEED_MAX_MM_S);
    r = clamp_f(r, SPEED_MIN_MM_S, SPEED_MAX_MM_S);

    s_tgt_l = (int16_t)l;
    s_tgt_r = (int16_t)r;
    MotorAdvance_SetTarget(s_tgt_l, s_tgt_r);
}

/* ============================== 对外接口 ============================== */

void Trace_Init(void)
{
    s_state      = TRACE_IDLE;
    s_mask       = 0u;
    s_seen_track = 0u;
    s_lost_cnt   = 0u;
    s_in_loss    = 0u;
    s_loss_ev    = 0u;
    s_blind_done = 0u;
    s_loss_ms    = 0u;
    s_loss_have  = 0u;
    s_gap_valid  = 0u;
    s_gap_d0l    = 0.0f;
    s_gap_d0r    = 0.0f;
    s_slow_arm   = 0u;
    s_slow_on    = 0u;
    s_slow_d0l   = 0.0f;
    s_slow_d0r   = 0.0f;

    s_black_edge = 0u;
    s_cross_idx  = 0u;
    s_finishing  = 0u;
    s_finish_d0l = 0.0f;
    s_finish_d0r = 0.0f;

    s_last_e     = 0.0f;
    s_e_lpf      = 0.0f;
    s_skip_d     = 0u;
    s_ramp_on    = 0u;
    s_ramp_d0l   = 0.0f;
    s_ramp_d0r   = 0.0f;
    s_yaw_same   = 0.0f;
    s_same_ok    = 0u;
    s_same_base  = 0.0f;
    s_same_has   = 0u;
    s_same_idx   = 0u;
    s_same_fill  = 0u;
    {
        uint8_t i;
        for (i = 0u; i < SAME_AVG_N; i++) { s_same_buf[i] = 0.0f; }
    }

    s_error      = 0.0f;
    s_steer      = 0.0f;
    s_tgt_l      = 0;
    s_tgt_r      = 0;

    s_blind      = 0u;
    s_yaw_ref    = 0.0f;
    s_blind_tick = 0u;
    s_reacq      = 0u;

    s_ab_on      = 0u;
    s_ab_d0l     = 0.0f;
    s_ab_d0r     = 0.0f;
    s_edge_watch = 0u;
    s_edge_last  = 0u;
    s_edge_cnt   = 0u;
    s_ft_on      = 0u;
    s_ft_target  = 0.0f;
    s_ft_tick    = 0u;
}

void Trace_Start(void)
{
    Trace_Init();
    s_state = TRACE_RUN;

    /* 记下发车这一刻的航向，作为整条赛道的绝对参考。
     * 所以：发车时车必须摆正 —— 这个值错了，盲走方向就会一起错。 */
    if (Mpu_IsReady() != 0u)
    {
        s_yaw_start = Mpu_GetYaw();
    }
}

void Trace_Stop(void)
{
    s_state = TRACE_IDLE;
    s_blind = 0u;
    s_tgt_l = 0;
    s_tgt_r = 0;
    MotorAdvance_Stop();
}

void Trace_Tick(void)
{
    uint8_t n;
    float   e;
    float   de;
    float   steer;

    s_mask = Gray_ReadMask();
    Mpu_Update();                   /* 陀螺仪同步更新（20ms 一拍） */

    if (s_state != TRACE_RUN)
    {
        goto mirror;                /* 未发车 / 已到终点：只读传感器供观察 */
    }

    n = popcount8(s_mask);

    /* ---------- 0) 盲走前减速 ----------
     * 过了第 5 个直角后开始计里程，走够 SLOW_START_MM 就降到慢速，
     * 一直保持到盲走段走完（s_blind_done）再恢复。 */
    if ((s_slow_arm != 0u) && (s_slow_on == 0u))
    {
        float dl = fabs_f(Motor_GetDistL() - s_slow_d0l);
        float dr = fabs_f(Motor_GetDistR() - s_slow_d0r);

        if (((dl + dr) * 0.5f) >= g_slow_start_mm)
        {
            s_slow_on = 1u;
        }
    }

    if (s_blind_done != 0u)
    {
        s_slow_on  = 0u;        /* 盲走结束，恢复正常速度 */
        s_slow_arm = 0u;
    }

    /* 出现过 1~4 路压线，说明已经离开发车区，进入正常赛道 */
    if ((n >= 1u) && (n <= 4u))
    {
        s_seen_track = 1u;
    }

    /* ---------- 1) 全黑线计数：第 N 次就是终点 ---------- */
    if (n >= 8u)
    {
        if (s_black_edge == 0u)         /* 上升沿：又压到一条全黑线 */
        {
            s_black_edge = 1u;
            if (s_cross_idx < 0xFFu) { s_cross_idx++; }

            if ((s_cross_idx == FINISH_CROSS_N) && (s_seen_track != 0u))
            {
                s_finishing  = 1u;
                s_finish_d0l = Motor_GetDistL();
                s_finish_d0r = Motor_GetDistR();
            }
        }
    }
    else
    {
        s_black_edge = 0u;
    }

    /* 终点冲刺：从第 N 次压线那一刻起，直行走 FINISH_RUN_MM 然后停车 */
    if (s_finishing != 0u)
    {
        float dl = fabs_f(Motor_GetDistL() - s_finish_d0l);
        float dr = fabs_f(Motor_GetDistR() - s_finish_d0r);

        if (((dl + dr) * 0.5f) >= FINISH_RUN_MM)
        {
            s_finishing = 0u;
            Trace_Stop();
            s_state = TRACE_FINISH;
            goto mirror;
        }

        s_error = 0.0f;
        s_steer = 0.0f;
        apply_steer(FINISH_SPEED_MM_S, 0.0f);   /* 直行 */
        goto mirror;
    }

    /* ---------- 2) 盲走中：陀螺仪保持航向 ---------- */
    if (s_blind != 0u)
    {
        /* 重新看到线 → 连续确认几拍后退出盲走 */
        if ((n >= 1u) && (n <= 4u))
        {
            if (s_reacq < 0xFFu) { s_reacq++; }
            if (s_reacq >= REACQ_TICKS)
            {
                s_blind = 0u;       /* 退出，本拍继续往下走正常循迹 */
                s_reacq = 0u;
                s_blind_done = 1u;  /* 盲走只走一次，之后丢线一律找线 */
                s_last_e = 0.0f;
                s_e_lpf  = 0.0f;    /* 清低通残留 */
                s_skip_d = 1u;      /* 第一拍不做微分 */

                /* 盲走出口：开始计里程，60cm 后开始盯边灯 */
                s_ab_on      = 1u;
                s_ab_d0l     = Motor_GetDistL();
                s_ab_d0r     = Motor_GetDistR();
                s_edge_watch = 0u;
                s_edge_last  = 0u;
                s_edge_cnt   = 0u;
            }
        }
        else
        {
            s_reacq = 0u;
        }

        if (s_blind != 0u)
        {
            if (s_blind_tick < 0xFFu) { s_blind_tick++; }

            /* 超时不再停车：一直保持航向直到重新看到线。
             * （原来这里会 Trace_Stop()，实测会在中途误停。） */

            /* 航向保持：纯 P，误差是角度
             *
             * 符号说明（很重要，写反了会"越修越歪"）：
             *   yaw 逆时针为正（Z 轴向上，右手定则）
             *   apply_steer 里 steer > 0 → 左轮快 → 车右转（顺时针）→ yaw 减小
             *   所以想让 yaw 往回走，steer 必须与"偏差"反号
             *   ★ 如果模块装反（Z 轴朝下），这里要再翻一次符号 */
            {
                float ye = s_yaw_ref - Mpu_GetYaw();
                float st;

                while (ye >  180.0f) { ye -= 360.0f; }
                while (ye < -180.0f) { ye += 360.0f; }

                st = clamp_f(-(KP_YAW * ye) + (g_kd_yaw_rate * Mpu_GetYawRate()),
                             -STEER_LIMIT_MM_S, STEER_LIMIT_MM_S);

                s_error = ye;
                s_steer = st;
                g_trace_ye = ye;
                apply_steer(BLIND_SPEED_MM_S, st);
                goto mirror;
            }
        }
    }

    /* ---------- 2b) 强制转向（环岛入口） ---------- */
    if (s_ft_on != 0u)
    {
        float ye = s_ft_target - Mpu_GetYaw();
        float st;

        while (ye >  180.0f) { ye -= 360.0f; }
        while (ye < -180.0f) { ye += 360.0f; }

        if (s_ft_tick < 0xFFu) { s_ft_tick++; }

        /* 转到位（误差 < 2.5°）或超时就交回 PD */
        if ((fabs_f(ye) < 2.5f) || (s_ft_tick >= ISLAND_TURN_TICKS))
        {
            s_ft_on   = 0u;
            s_skip_d  = 1u;                      /* 下一拍不做微分，避免假跳变 */
            s_ramp_on = 1u;                      /* 开始把速度平滑升回 BASE */
            s_ramp_d0l = Motor_GetDistL();
            s_ramp_d0r = Motor_GetDistR();
            /* 落下去走正常循迹 */
        }
        else
        {
            st = clamp_f(-(KP_YAW * ye) + (g_kd_yaw_rate * Mpu_GetYawRate()),
                             -STEER_LIMIT_MM_S, STEER_LIMIT_MM_S);
            s_error = ye;
            s_steer = st;
            g_trace_ye = ye;
            apply_steer(ISLAND_TURN_SPEED, st);
            goto mirror;
        }
    }

    /* ---------- 2c) 盲走出来 60cm 后：盯边灯，第 2 次上升沿强制转 30° ---------- */
    if (s_ab_on != 0u)
    {
        if (s_edge_watch == 0u)
        {
            float dl = fabs_f(Motor_GetDistL() - s_ab_d0l);
            float dr = fabs_f(Motor_GetDistR() - s_ab_d0r);

            if (((dl + dr) * 0.5f) >= ISLAND_WATCH_MM)
            {
                s_edge_watch = 1u;
                s_edge_last  = 0u;
                s_edge_cnt   = 0u;
            }
        }
        else
        {
            uint8_t bit = (uint8_t)((s_mask >> ISLAND_EDGE_BIT) & 0x01u);

            if ((bit != 0u) && (s_edge_last == 0u))     /* 上升沿才算一次 */
            {
                if (s_edge_cnt < 0xFFu) { s_edge_cnt++; }

                if (s_edge_cnt >= ISLAND_EDGE_N)
                {
                    s_edge_watch = 0u;
                    s_ab_on      = 0u;
                    s_ft_on      = 1u;
                    s_ft_tick    = 0u;
                    s_ft_target  = norm180(Mpu_GetYaw() + (ISLAND_TURN_SIGN * ISLAND_TURN_DEG));
                }
            }
            s_edge_last = bit;
        }
    }

    /* ---------- 3) 丢线：全白 ---------- */
    if (n == 0u)
    {
        /* 还没见过线：说明没上赛道 / 模块悬空 / 模块没供电。
         * 这种情况绝对不能"找线"，否则一上电车就自己冲出去。 */
        if (s_seen_track == 0u)
        {
            s_error = 0.0f;
            s_steer = 0.0f;
            s_tgt_l = 0;
            s_tgt_r = 0;
            s_lost_cnt = 0u;
            MotorAdvance_SetTarget(0, 0);
            goto mirror;
        }

        /* ---- 丢线处理：按"第几次丢线"写死，不再靠误差大小判断 ---- */
        if (s_in_loss == 0u)              /* 一次丢线的上升沿 */
        {
            uint32_t now  = HAL_GetTick();
            uint8_t  pass = 1u;

            s_in_loss = 1u;

            /* 第 5 个弯之后的那 1 米是直道，没有弯道 —— 这期间的丢线一律不计数。
             * (s_slow_arm=1 表示已数到第 5 个弯；s_slow_on=0 表示还没走够那 1 米) */
            if ((s_slow_arm != 0u) && (s_slow_on == 0u))
            {
                pass = 0u;
            }

            /* 门槛：已经数到第 6 个弯时，必须再走够 CORNER6_GAP_MM
             * 才允许数出第 7 个弯，避免同一个弯被数两次。 */
            if ((pass != 0u) && (s_loss_ev == 6u) && (s_gap_valid != 0u))
            {
                float dl = fabs_f(Motor_GetDistL() - s_gap_d0l);
                float dr = fabs_f(Motor_GetDistR() - s_gap_d0r);

                if (((dl + dr) * 0.5f) < g_corner6_gap_mm) { pass = 0u; }
            }

            /* 500ms 消抖：离上次计数太近的丢线不算，防止同一个弯被数好几次。
             * 被过滤掉的这次，动作仍然按当前 s_loss_ev 走，不影响找线。 */
            if (pass != 0u)
            {
                if ((s_loss_have != 0u) &&
                    ((now - s_loss_ms) < (uint32_t)g_loss_debounce_ms))
                {
                    pass = 0u;
                }
            }

            if (pass != 0u)
            {
                if (s_loss_ev < 0xFFu) { s_loss_ev++; }
                s_loss_ms   = now;
                s_loss_have = 1u;

                /* 第 5 个直角之后开始计里程，为盲走前减速做准备 */
                if ((s_loss_ev == SLOW_AFTER_CORNER) && (s_slow_arm == 0u) && (s_slow_on == 0u))
                {
                    s_slow_arm = 1u;
                    s_slow_d0l = Motor_GetDistL();
                    s_slow_d0r = Motor_GetDistR();
                }

                /* 数到第 6 个弯：记下里程，开始等 20cm 门槛 */
                if (s_loss_ev == 6u)
                {
                    s_gap_valid = 1u;
                    s_gap_d0l   = Motor_GetDistL();
                    s_gap_d0r   = Motor_GetDistR();
                }
            }
        }

        if ((s_loss_ev <= CORNER_COUNT) || (s_blind_done != 0u))
        {
            /* 前 7 次 = 直角弯；盲走段走过之后再丢线也一律按找线处理，
             * 否则第二个岛的钝角岔口会被误判成又一次盲走。 */
            steer  = (s_last_e >= 0.0f) ? SEARCH_STEER_MM_S : -SEARCH_STEER_MM_S;
            s_error = s_last_e;
            s_steer = steer;
            apply_steer(cap_slow(BASE_MM_S), steer);
            goto mirror;
        }

        /* 第 8 次起 = 盲走（断口），陀螺仪保持航向 */
        if (Mpu_IsReady() == 0u)          /* 陀螺仪没就绪就退回打舵找线 */
        {
            steer  = (s_last_e >= 0.0f) ? SEARCH_STEER_MM_S : -SEARCH_STEER_MM_S;
            s_error = s_last_e;
            s_steer = steer;
            apply_steer(cap_slow(BASE_MM_S), steer);
            goto mirror;
        }

        s_blind      = 1u;
        s_ab_on      = 0u;      /* 重新进盲走，环岛监视作废 */
        s_edge_watch = 0u;
        /* 目标 = 发车航向 + 偏移角（赛道绝对方向），
         * 而不是"丢线瞬间的车头方向" —— 车进断口时可能正带着修正角。 */
        /* 有第 3~4 弯之间那段直道的航向就用它（漂移窗口短 3 倍），
         * 没有就退回"发车航向 + 偏移角"。
         * BLIND_OFFSET_DEG 仍然当微调旋钮：填 180 = 不修正。 */
        if (s_same_ok != 0u)
        {
            s_yaw_ref = norm180(s_yaw_same + (BLIND_OFFSET_DEG - 180.0f));
        }
        else
        {
            s_yaw_ref = norm180(s_yaw_start + BLIND_OFFSET_DEG);
        }
        s_blind_tick = 0u;
        s_reacq      = 0u;
        s_error      = 0.0f;
        s_steer      = 0.0f;
        g_trace_ye   = s_yaw_ref - Mpu_GetYaw();   /* 给 Watch 看：靠近 0 说明偏移角对 */
        apply_steer(BLIND_SPEED_MM_S, 0.0f);
        goto mirror;
    }
    s_lost_cnt = 0u;
    s_in_loss  = 0u;

    /* ---------- 4) 宽黑（十字 / 圆环 / 发车区）：直行，冻结误差 ---------- */
    if (n >= WIDE_COUNT)
    {
        s_error = s_last_e;     /* 不更新，避免重心算出来是 0 造成误判 */
        s_steer = 0.0f;
        apply_steer(cap_slow(BASE_MM_S), 0.0f);
        goto mirror;
    }

    /* ---------- 5) 正常循迹 ---------- */

    /* 第 3 个弯之后到第 4 个弯之间那段直道，方向和盲走段平行同向 ——
     * 在这里把航向录下来当盲走目标。
     *   · 环形缓冲只保留最近 SAME_AVG_N 个样本 → 用的是第 4 个弯之前
     *     最后那段已经走稳的航向，离盲走入口最近
     *   · 求平均 → 把单点噪声和残余摆动平均掉
     *   · 只在线跟得好、且车确实在直行（角速度小）时才采 */
    if ((s_loss_ev == 3u) && (n >= 1u) && (n <= 4u) &&
        (fabs_f(calc_error(s_mask)) < 1.5f) &&
        (fabs_f(Mpu_GetYawRate()) < 20.0f))
    {
        float   y = Mpu_GetYaw();
        float   sum = 0.0f;
        uint8_t i;

        if (s_same_has == 0u)
        {
            s_same_base = y;          /* 第一个样本定基准，避开 ±180 折返 */
            s_same_has  = 1u;
        }

        s_same_buf[s_same_idx] = norm180(y - s_same_base);
        s_same_idx = (uint8_t)((s_same_idx + 1u) % SAME_AVG_N);
        if (s_same_fill < SAME_AVG_N) { s_same_fill++; }

        for (i = 0u; i < s_same_fill; i++) { sum += s_same_buf[i]; }

        s_yaw_same = norm180(s_same_base + (sum / (float)s_same_fill));
        s_same_ok  = 1u;
    }

    e = calc_error(s_mask);

#if (TRACE_REVERSE != 0)
    e = -e;
#endif

    /* 一阶低通：滤掉量化跳变，KD 才能用大而不抖。alpha >= 1 视为关闭 */
    if (g_lpf_alpha < 0.999f)
    {
        s_e_lpf += g_lpf_alpha * (e - s_e_lpf);
    }
    else
    {
        s_e_lpf = e;
    }

    /* 刚退出强制转向的第一拍不做微分：否则 de = e - 0，会假跳一下 */
    if (s_skip_d != 0u)
    {
        de       = 0.0f;
        s_skip_d = 0u;
    }
    else
    {
        de = s_e_lpf - s_last_e;
    }
    s_last_e = s_e_lpf;

    steer   = (KP_TRACE * s_e_lpf) + (KD_TRACE * de);

    /* 环岛转向结束后的速度渐变：从 ISLAND_TURN_SPEED 线性升到 BASE，
     * 升满需要走 ISLAND_RAMP_MM 毫米。按里程而不是时间，换速度也一致。 */
    {
        float base = BASE_MM_S;

        if (s_ramp_on != 0u)
        {
            float dl = fabs_f(Motor_GetDistL() - s_ramp_d0l);
            float dr = fabs_f(Motor_GetDistR() - s_ramp_d0r);
            float p;

            if (ISLAND_RAMP_MM <= 0.5f)
            {
                s_ramp_on = 0u;                 /* 不渐变 */
            }
            else
            {
                p = ((dl + dr) * 0.5f) / ISLAND_RAMP_MM;
                if (p >= 1.0f)
                {
                    s_ramp_on = 0u;             /* 升满了 */
                }
                else
                {
                    base = ISLAND_TURN_SPEED
                         + (BASE_MM_S - ISLAND_TURN_SPEED) * p;
                }
            }
        }

        s_error = e;
        s_steer = steer;
        apply_steer(cap_slow(base), steer);
    }

mirror:
    /* 同步给 Keil Watch（Watch 不能调函数，只能看变量） */
    g_trace_mask  = s_mask;
    g_trace_err   = s_error;
    g_trace_steer = s_steer;
    g_trace_tgtl  = s_tgt_l;
    g_trace_tgtr  = s_tgt_r;
    g_trace_blind = s_blind;
    g_trace_cross = s_cross_idx;
    g_trace_fin   = s_finishing;
    g_trace_ab    = s_ab_on;
    g_trace_edge  = s_edge_cnt;
    g_trace_ft    = s_ft_on;
    g_trace_lev   = s_loss_ev;
    g_trace_dist  = (Motor_GetDistL() + Motor_GetDistR()) * 0.5f;
}

/* ============================== 观测接口 ============================== */

TraceState Trace_GetState(void)  { return s_state;  }
uint8_t    Trace_GetMask(void)   { return s_mask;   }
float      Trace_GetError(void)  { return s_error;  }
float      Trace_GetSteer(void)  { return s_steer;  }
int16_t    Trace_GetTargetL(void){ return s_tgt_l;  }
int16_t    Trace_GetTargetR(void){ return s_tgt_r;  }
