/**
  ******************************************************************************
  * @file    Motor_advance.c
  * @brief   双轮速度环：前馈 + PI，20ms 一拍
  *
  *  每个轮子一套独立控制器，互不影响（右边跑得快不会把左边拖慢）。
  *
  *  控制律：
  *      duty = FF(v_target) + Kp*e + Ki*∫e
  *      FF(v) = v / FF_SPEED_AT_FULL * 100
  *
  *  - 前馈把"目标速度"直接换算成占空比，负责大头；
  *  - PI 只修电池掉压、地面摩擦、左右电机个体差异这些残差；
  *  - 条件积分：输出已饱和且误差还在往饱和方向推时，停止积分，防止 windup；
  *  - 目标斜坡：避免一起步就打满占空比导致轮胎打滑/电流冲击。
  ******************************************************************************
  */

#include "Motor_advance.h"
#include "Motor_base.h"

/* ==================== 可调参数（调速度环只动这一段） ==================== */

// 前馈系数：100% 占空比时的空载速度，单位 mm/s。
#define FF_SPEED_AT_FULL     1275

/* PI 增益。误差单位 mm/s，输出单位 % 占空比 */
#define KP_DUTY              0.10f
#define KI_DUTY              0.10f

/* 积分限幅（折成占空比 %） */
#define INTEG_MAX_DUTY       40.0f

/* 目标斜坡：每拍(10ms)最多变化多少 mm/s。0 = 不限制
 * 30 mm/s per 10ms = 3000 mm/s²（和 60 per 20ms 等价） */
#define TARGET_SLEW_PER_TICK 30.0f

/* 目标速度上限 mm/s，防止误传大数 */
#define TARGET_MAX_MM_S      1200

/* 实测速度一阶低通系数：1.0 = 不滤波（现在分辨率够用），0.5 = 中等 */
#define MEAS_LPF_ALPHA       1.0f

/* 停车死区：目标为 0 且实测低于这个速度时，硬清积分并输出 0，防止蠕动 */
#define STOP_SPEED_MM_S      8.0f

/* 堵转保护：占空比 >= 60% 且速度 < 50mm/s，连续 25 拍(500ms) -> 判堵转 */
#define STALL_DUTY           60.0f
#define STALL_SPEED          50.0f
#define STALL_TICKS          50

/* 控制周期（秒） */
#define CTRL_S               (CTRL_PERIOD_MS * 0.001f)

/* 积分项上限，单位 mm/s·s */
#define INTEG_LIMIT          (INTEG_MAX_DUTY / KI_DUTY)

/* ============================== 内部状态 ============================== */

typedef struct
{
    float    target_raw;   /* 外部设定的目标 mm/s */
    float    target;       /* 斜坡后的目标 mm/s */
    float    speed;        /* 滤波后实测 mm/s */
    float    raw_speed;    /* 本拍实测 mm/s（未滤波） */
    float    integ;        /* 积分项，单位 mm/s·s */
    float    duty;         /* 本拍输出 % */
    uint16_t stall_cnt;
} Wheel_t;

static Wheel_t s_w[2];          /* s_w[0] = 左轮, s_w[1] = 右轮 */
static uint8_t s_stalled = 0;

/* ============================== 小工具 ============================== */

static float fabs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float clamp_f(float v, float lim)
{
    if (v >  lim) { return  lim; }
    if (v < -lim) { return -lim; }
    return v;
}

static float slew_to(float cur, float want, float step)
{
    float d = want - cur;

    if (step <= 0.0f) { return want; }
    if (d >  step)    { return cur + step; }
    if (d < -step)    { return cur - step; }
    return want;
}

static void wheel_reset(Wheel_t *w)
{
    w->target_raw = 0.0f;
    w->target     = 0.0f;
    w->speed      = 0.0f;
    w->raw_speed  = 0.0f;
    w->integ      = 0.0f;
    w->duty       = 0.0f;
    w->stall_cnt  = 0;
}

static void wheel_set_target(Wheel_t *w, int16_t mm_s)
{
    float v = (float)mm_s;

    if (v >  (float)TARGET_MAX_MM_S) { v =  (float)TARGET_MAX_MM_S; }
    if (v < -(float)TARGET_MAX_MM_S) { v = -(float)TARGET_MAX_MM_S; }
    w->target_raw = v;
}

/* 一个轮子走一拍：输入本拍实测速度(mm/s)，返回本拍占空比(%) */
static float wheel_step(Wheel_t *w, float meas)
{
    float err;
    float ff;
    float out;

    /* 1) 目标斜坡 */
    w->target = slew_to(w->target, w->target_raw, TARGET_SLEW_PER_TICK);

    /* 2) 实测低通 */
    w->raw_speed = meas;
    w->speed += MEAS_LPF_ALPHA * (meas - w->speed);

    /* 3) 前馈 */
    ff = (w->target / FF_SPEED_AT_FULL) * 100.0f;

    /* 4) PI + 条件积分抗饱和 */
    err = w->target - w->speed;

    out = ff + KP_DUTY * err + KI_DUTY * w->integ;
    if (!(((out >  100.0f) && (err > 0.0f)) ||
          ((out < -100.0f) && (err < 0.0f))))
    {
        w->integ += err * CTRL_S;
        w->integ  = clamp_f(w->integ, INTEG_LIMIT);
    }

    out = ff + KP_DUTY * err + KI_DUTY * w->integ;
    out = clamp_f(out, 100.0f);

    /* 5) 停车死区：目标 0 且已经基本停下，就彻底归零，不留积分残余 */
    if ((fabs_f(w->target_raw) < 0.5f) && (fabs_f(w->speed) < STOP_SPEED_MM_S))
    {
        w->integ = 0.0f;
        out      = 0.0f;
    }

    w->duty = out;
    return out;
}

/* 返回 1 表示这一路已经堵转够久 */
static uint8_t stall_check(Wheel_t *w)
{
    if ((fabs_f(w->duty) >= STALL_DUTY) &&
        (fabs_f(w->speed) < STALL_SPEED) &&
        (fabs_f(w->target_raw) >= (STALL_SPEED * 2.0f)))
    {
        if (w->stall_cnt < 0xFFFFu) { w->stall_cnt++; }
    }
    else
    {
        w->stall_cnt = 0;
    }

    return (w->stall_cnt >= STALL_TICKS) ? 1u : 0u;
}

/* ============================== 对外接口 ============================== */

void MotorAdvance_Init(void)
{
    wheel_reset(&s_w[0]);
    wheel_reset(&s_w[1]);
    s_stalled = 0;
    Motor_SetSpeed(0, 0);
}

void MotorAdvance_SetTarget(int16_t mm_s_l, int16_t mm_s_r)
{
    wheel_set_target(&s_w[0], mm_s_l);
    wheel_set_target(&s_w[1], mm_s_r);
}

void MotorAdvance_Tick(void)
{
    float   dl;
    float   dr;
    uint8_t fL;
    uint8_t fR;

    dl = wheel_step(&s_w[0], Motor_GetSpeedL());
    dr = wheel_step(&s_w[1], Motor_GetSpeedR());

    fL = stall_check(&s_w[0]);
    fR = stall_check(&s_w[1]);
    if ((fL != 0u) || (fR != 0u)) { s_stalled = 1u; }

    if (s_stalled != 0u)
    {
        dl = 0.0f;
        dr = 0.0f;
        s_w[0].integ = 0.0f;
        s_w[1].integ = 0.0f;
        s_w[0].duty  = 0.0f;
        s_w[1].duty  = 0.0f;
    }

    Motor_SetSpeed((int)dl, (int)dr);
}

void MotorAdvance_Stop(void)
{
    s_w[0].target_raw = 0.0f;
    s_w[1].target_raw = 0.0f;
    s_w[0].target     = 0.0f;
    s_w[1].target     = 0.0f;
    s_w[0].integ      = 0.0f;
    s_w[1].integ      = 0.0f;
    s_w[0].duty       = 0.0f;
    s_w[1].duty       = 0.0f;
    Motor_SetSpeed(0, 0);
}

int16_t MotorAdvance_GetSpeedL(void) { return (int16_t)s_w[0].speed; }
int16_t MotorAdvance_GetSpeedR(void) { return (int16_t)s_w[1].speed; }
int16_t MotorAdvance_GetDutyL(void)  { return (int16_t)s_w[0].duty;  }
int16_t MotorAdvance_GetDutyR(void)  { return (int16_t)s_w[1].duty;  }
int16_t MotorAdvance_GetTargetL(void){ return (int16_t)s_w[0].target; }
int16_t MotorAdvance_GetTargetR(void){ return (int16_t)s_w[1].target; }

uint8_t MotorAdvance_IsStalled(void)
{
    return s_stalled;
}

void MotorAdvance_ClearFault(void)
{
    s_stalled        = 0u;
    s_w[0].stall_cnt = 0;
    s_w[1].stall_cnt = 0;
}
