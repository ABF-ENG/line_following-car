#include "Motor_base.h"
static int32_t s_lastL = 0;//上一次编码读数
static int16_t s_lastR = 0;
static volatile int32_t s_dL = 0;//当前周期编码读数
static volatile int16_t s_dR = 0;
static float   s_distL = 0.0f, s_distR = 0.0f;//总共走过的距离
volatile uint8_t g_ctrl_tick = 0;

static void set_pwm(uint32_t ch, int duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1u;
    if (duty < 0)   duty = 0;
    if (duty > 100) duty = 100;
    __HAL_TIM_SET_COMPARE(&htim1, ch, (uint32_t)duty * arr / 100u);
}

void Motor_Init(void)
{
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
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
    } else if (duty_r < 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
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
}

int32_t Motor_GetDeltaL(void) { return s_dL; }
int32_t Motor_GetDeltaR(void) { return s_dR; }

float Motor_GetSpeedL(void) { return (float)s_dL * MM_S_PER_CNT; }
float Motor_GetSpeedR(void) { return (float)s_dR * MM_S_PER_CNT; }

float Motor_GetDistL(void) { return s_distL; }
float Motor_GetDistR(void) { return s_distR; }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
		Motor_EncoderUpdate();
		g_ctrl_tick=1;
    }
}