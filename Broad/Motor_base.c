#include "Motor_base.h"
static int32_t s_lastL = 0;//上一次编码读数
static int16_t s_lastR = 0;
static volatile int32_t s_dL = 0;//当前周期编码读数
static volatile int16_t s_dR = 0;
static float   s_distL = 0.0f, s_distR = 0.0f;//总共走过的距离
volatile uint8_t g_ctrl_tick = 0;

/* ======================= 急停（PC0 按键，按下 = 低电平） =======================
 *
 *  行为：一旦按下 → 立刻短路刹车 + 锁存，之后任何 Motor_SetSpeed() 都被否决，
 *        直到显式调用 Motor_ClearEStop()。
 *
 *  为什么在两处都查：
 *    1) TIM6 中断里（Motor_EncoderUpdate）—— 主循环卡死也照样停
 *    2) Motor_SetSpeed 入口 —— 防止主循环在中断之后又把输出写回去
 */
#define ESTOP_PORT        GPIOC
#define ESTOP_PIN         GPIO_PIN_0

static volatile uint8_t s_estop_latched = 0;

static void set_pwm(uint32_t ch, int duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1u;
    if (duty < 0)   duty = 0;
    if (duty > 100) duty = 100;
    __HAL_TIM_SET_COMPARE(&htim1, ch, (uint32_t)duty * arr / 100u);
}

/* PC0 引脚初始化（CubeMX 里已经配了也没关系，重复配置无害） */
static void estop_pin_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    g.Pin  = ESTOP_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;       /* 上拉：松开 = 1，按下 = 0 */
    HAL_GPIO_Init(ESTOP_PORT, &g);
}

/* 短路刹车：IN1=IN2=1 让电机两端短接，比滑行停得快得多 */
static void estop_brake(void)
{
    HAL_GPIO_WritePin(GPIOE,
        GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
    set_pwm(TIM_CHANNEL_1, 0);
    set_pwm(TIM_CHANNEL_2, 0);
}

/* 查一次按键：按下则锁存并刹车。返回 1 表示当前处于急停状态 */
static uint8_t estop_poll(void)
{
    if (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET)
    {
        s_estop_latched = 1u;       /* 按下 → 锁存，松手也不恢复 */
    }

    if (s_estop_latched != 0u)
    {
        estop_brake();
        return 1u;
    }
    return 0u;
}


void Motor_Init(void)
{
    estop_pin_init();               /* 急停按键引脚 */

    HAL_GPIO_WritePin(GPIOE,
        GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    set_pwm(TIM_CHANNEL_1, 0);
    set_pwm(TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    s_lastL = 0;  s_lastR = 0;
    s_dL = 0;     s_dR = 0;
    s_distL = 0.0f;  s_distR = 0.0f;
}

void Motor_SetSpeed(int duty_l, int duty_r)
{
    /* 急停优先：锁存期间任何速度指令都不执行 */
    if (estop_poll() != 0u)
    {
        return;
    }

    if (duty_l > 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    } else if (duty_l < 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        duty_l = -duty_l;
    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    }
    set_pwm(TIM_CHANNEL_1, duty_l);

    if (duty_r > 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
    } else if (duty_r < 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
        duty_r = -duty_r;
    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    set_pwm(TIM_CHANNEL_2, duty_r);
}

void Motor_EncoderUpdate(void)
{
    int32_t nowL = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t nowR = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    s_dL = -(nowL - s_lastL);
    s_dR =  (nowR - s_lastR);

    s_lastL = nowL;
    s_lastR = nowR;

    s_distL += (float)s_dL * MM_PER_COUNT;
    s_distR += (float)s_dR * MM_PER_COUNT;

    /* 急停在这里也查一次：跑在 TIM6 中断里，即使主循环卡死也能停车 */
    (void)estop_poll();
}

int32_t Motor_GetDeltaL(void) { return s_dL; }
int32_t Motor_GetDeltaR(void) { return s_dR; }

float Motor_GetSpeedL(void) { return (float)s_dL * MM_S_PER_CNT; }
float Motor_GetSpeedR(void) { return (float)s_dR * MM_S_PER_CNT; }

float Motor_GetDistL(void) { return s_distL; }
float Motor_GetDistR(void) { return s_distR; }

/* ============================== 急停接口 ============================== */

/* 读引脚：1 = 按键当前按下（不改变锁存状态） */
uint8_t Motor_IsEStopPressed(void)
{
    return (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
}

/* 1 = 已锁存急停（按过一次就一直为 1，直到 ClearEStop） */
uint8_t Motor_IsEStopLatched(void)
{
    return s_estop_latched;
}

/* 清除急停锁存。只有按键已经松开时才允许清，避免按住不放时被反复清掉 */
void Motor_ClearEStop(void)
{
    if (Motor_IsEStopPressed() == 0u)
    {
        s_estop_latched = 0u;
        Motor_SetSpeed(0, 0);       /* 清完之后恢复滑行状态 */
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
		Motor_EncoderUpdate();
		g_ctrl_tick=1;
    }
}