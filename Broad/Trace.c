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

/* ======================= 可调参数（调循迹只动这一段） ======================= */

/* 正常循迹的基础车速 mm/s */
#define BASE_MM_S          400.0f

/* 比例增益：每 1 个误差单位给多少 mm/s 的差速。误差范围 ±7 */
#define KP_TRACE            30.0f

/* 微分增益：作用在"每拍误差变化量"上，抑制摆动。太大对传感器跳变敏感
 * ★ 它没有除以采样周期，等效阻尼 = KD × CTRL_PERIOD_MS。
 *   周期 20ms → 10ms 时等效阻尼砍半，所以 KD 要翻倍（15 → 30）补回来。 */
#define KD_TRACE            15.0f

/* 车轮速度限幅：内侧轮别停死（停死等于原地转，容易拖出赛道） */
#define SPEED_MIN_MM_S      60.0f
#define SPEED_MAX_MM_S     900.0f

/* 丢线时的找线差速 mm/s */
#define SEARCH_STEER_MM_S  340.0f

/* 判为"宽黑线"（十字/圆环/终点）所需的最少探头数，8 路里 5 路以上算宽 */
#define WIDE_COUNT          6u

/* ============================ 起终点黑线 ============================ */

/* 盲走之后 → 环岛触发 */
#define ISLAND_WATCH_MM     600.0f   /* 从盲走出口再走多远开始盯边灯 */
#define ISLAND_EDGE_BIT     7u       /* 右边第 1 颗灯 = X8 = bit7（装反了就改成 0） */
#define ISLAND_EDGE_N       2u       /* 第几次上升沿才动作 */
#define ISLAND_TURN_DEG     30.0f    /* 强制转多少度 */
#define ISLAND_TURN_SIGN    (-1.0f)  /* +1 = 逆时针(左转)，-1 = 顺时针(右转) */
#define ISLAND_TURN_SPEED   220.0f   /* 转向时的轮速 mm/s */
#define ISLAND_TURN_TICKS   40u      /* 转向超时 40×20ms = 800ms */

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

/* ============================ 盲走（断线） ============================ */

/* 盲走速度 mm/s —— 比循迹略慢，冲过断口更可控 */
#define BLIND_SPEED_MM_S    500.0f

/* 航向保持增益：每 1° 偏差给多少 mm/s 的差速 */
#define KP_YAW              3.0f

/* 航向修正的差速限幅，防止一上来猛打 */
#define STEER_LIMIT_MM_S    200.0f

/* 盲走超时：150 拍 = 3s，400mm/s × 3s = 1.2m，足够过任何断口 */
#define BLIND_TIMEOUT_TICKS 150u

/* 判"断口"还是"直角弯"：丢线瞬间 |误差| 小于这个值才算断口。
 *   断口：线在正中时突然没了        → |e| ≈ 0~2   → 盲走保持航向
 *   直角：线先跑到最外侧才没了      → |e| ≈ 5~7   → 打舵找线
 * 这个阈值要在 2~4 之间，别设太大，否则直角会被误判成断口。 */
#define BLIND_ENTER_E       3.0f

/* 盲走的目标航向 = 发车时航向 + 这个偏移角（度）
 *   180 = 盲走段和发车段平行且方向相反（按你这条赛道）
 *   如果实测发现盲走段其实和发车段同向，改成 0
 * 进盲走那一下看 g_trace_ye：接近 0 说明偏移角对了；接近 180 说明该改成 0。 */
#define BLIND_OFFSET_DEG    180.0f

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

/* ---- 起终点黑线 ---- */
static uint8_t    s_black_edge  = 0u;    /* 上一拍是否全黑（检测上升沿） */
static uint8_t    s_cross_idx   = 0u;    /* 已经压到几次全黑线 */
static uint8_t    s_finishing   = 0u;    /* 1 = 正在终点冲刺段 */
static float      s_finish_d0l  = 0.0f;  /* 冲刺起点（左轮里程） */
static float      s_finish_d0r  = 0.0f;  /* 冲刺起点（右轮里程） */

static float      s_last_e      = 0.0f;  /* 上一次的误差（丢线/宽黑时保持） */
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

    s_black_edge = 0u;
    s_cross_idx  = 0u;
    s_finishing  = 0u;
    s_finish_d0l = 0.0f;
    s_finish_d0r = 0.0f;

    s_last_e     = 0.0f;
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
                s_last_e = 0.0f;

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

                st = clamp_f(-KP_YAW * ye, -STEER_LIMIT_MM_S, STEER_LIMIT_MM_S);

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
            s_last_e  = 0.0f;
            /* 落下去走正常循迹 */
        }
        else
        {
            st = clamp_f(-KP_YAW * ye, -STEER_LIMIT_MM_S, STEER_LIMIT_MM_S);
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

        /* 丢线分两种情况，处理完全不同：
         *   断口  ：线在正中时突然没了 → 陀螺仪保持航向直行
         *   直角弯：线跑到最外侧才没了 → 朝最后看到的方向打舵找线
         * 判据就是丢线那一刻的 |误差|。 */
        if ((fabs_f(s_last_e) < BLIND_ENTER_E) && (Mpu_IsReady() != 0u))
        {
            s_blind      = 1u;
            s_ab_on      = 0u;      /* 重新进盲走，环岛监视作废 */
            s_edge_watch = 0u;
            /* 目标 = 发车航向 + 偏移角（赛道绝对方向），
             * 而不是"丢线瞬间的车头方向" —— 车进断口时可能正带着修正角。 */
            s_yaw_ref    = norm180(s_yaw_start + BLIND_OFFSET_DEG);
            s_blind_tick = 0u;
            s_reacq      = 0u;
            s_error      = 0.0f;
            s_steer      = 0.0f;
            g_trace_ye   = s_yaw_ref - Mpu_GetYaw();   /* 给 Watch 看：靠近 0 说明偏移角对 */
            apply_steer(BLIND_SPEED_MM_S, 0.0f);
            goto mirror;
        }

        if (s_lost_cnt < 0xFFu) { s_lost_cnt++; }

        /* 找线超时也不再停车：一直朝最后看到线的方向找。
         * （原来这里 1 秒就 Trace_Stop()，直角弯上会误停。） */

        /* 朝最后看到线的方向找：线在右(e>0)就往右打舵 */
        steer  = (s_last_e >= 0.0f) ? SEARCH_STEER_MM_S : -SEARCH_STEER_MM_S;
        s_error = s_last_e;
        s_steer = steer;
        apply_steer(BASE_MM_S, steer);
        goto mirror;
    }
    s_lost_cnt = 0u;

    /* ---------- 4) 宽黑（十字 / 圆环 / 发车区）：直行，冻结误差 ---------- */
    if (n >= WIDE_COUNT)
    {
        s_error = s_last_e;     /* 不更新，避免重心算出来是 0 造成误判 */
        s_steer = 0.0f;
        apply_steer(BASE_MM_S, 0.0f);
        goto mirror;
    }

    /* ---------- 5) 正常循迹 ---------- */
    e = calc_error(s_mask);

#if (TRACE_REVERSE != 0)
    e = -e;
#endif

    de = e - s_last_e;
    s_last_e = e;

    steer   = (KP_TRACE * e) + (KD_TRACE * de);
    s_error  = e;
    s_steer  = steer;

    apply_steer(BASE_MM_S, steer);

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
}

/* ============================== 观测接口 ============================== */

TraceState Trace_GetState(void)  { return s_state;  }
uint8_t    Trace_GetMask(void)   { return s_mask;   }
float      Trace_GetError(void)  { return s_error;  }
float      Trace_GetSteer(void)  { return s_steer;  }
int16_t    Trace_GetTargetL(void){ return s_tgt_l;  }
int16_t    Trace_GetTargetR(void){ return s_tgt_r;  }
