
#include "Gray.h"
#include "main.h"

/* 切地址后等多久再读（µs）。厂商例程是 50µs。
 * 想更快可以降到 10~20，但先按 50 跑通再说。 */
#define GRAY_SETTLE_US   50u

/* 1 = 每路连采 3 次取多数（电机干扰大时打开），耗时 ×3；0 = 采一次 */
#define GRAY_VOTE3       0u

/* ------------------------------------------------------------------------- */

/* 用 DWT 周期计数器做微秒延时：不能在 20ms 控制周期里用 HAL_Delay(1)，
 * 那样 8 路就是 8ms，直接吃掉半个周期。 */
static void gray_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);

    while ((DWT->CYCCNT - start) < ticks)
    {
        /* 空转 */
    }
}

/* 一次地址切换：BSRR 高 16 位清零、低 16 位置位，一条指令搞定三个脚 */
static void gray_select(uint8_t ch)
{
    uint32_t set = 0u;
    uint32_t clr = 0u;

    if ((ch & 0x01u) != 0u) { set |= GRAY_AD0_PIN; } else { clr |= GRAY_AD0_PIN; }
    if ((ch & 0x02u) != 0u) { set |= GRAY_AD1_PIN; } else { clr |= GRAY_AD1_PIN; }
    if ((ch & 0x04u) != 0u) { set |= GRAY_AD2_PIN; } else { clr |= GRAY_AD2_PIN; }

    GRAY_GPIO_ADDR->BSRR = set | (clr << 16);
}

static uint8_t gray_sample(void)
{
#if (GRAY_VOTE3 != 0u)
    uint8_t v = 0u;

    if ((GRAY_GPIO_OUT->IDR & GRAY_OUT_PIN) != 0u) { v++; }
    gray_delay_us(5u);
    if ((GRAY_GPIO_OUT->IDR & GRAY_OUT_PIN) != 0u) { v++; }
    gray_delay_us(5u);
    if ((GRAY_GPIO_OUT->IDR & GRAY_OUT_PIN) != 0u) { v++; }

    return (v >= 2u) ? 1u : 0u;
#else
    return ((GRAY_GPIO_OUT->IDR & GRAY_OUT_PIN) != 0u) ? 1u : 0u;
#endif
}

/* ------------------------------------------------------------------------- */

void Gray_Init(void)
{
    /* PD0~PD2 输出 / PD3 输入的 GPIO 配置由 CubeMX 生成的 MX_GPIO_Init() 完成，
     * 这里只把 DWT 周期计数器打开，供 micro 级延时用。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* 上电先选中 X1，避免地址线悬空 */
    gray_select(0u);
}

uint8_t Gray_ReadMask(void)
{
    uint8_t ch;
    uint8_t mask = 0u;

    for (ch = 0u; ch < 8u; ch++)
    {
        gray_select(ch);
        gray_delay_us(GRAY_SETTLE_US);

        if (gray_sample() != 0u)
        {
            mask |= (uint8_t)(1u << ch);   /* bit0 = X1 … bit7 = X8 */
        }
    }

    return mask;
}

void Gray_MaskToStr(uint8_t mask, char *out)
{
    uint8_t i;

    for (i = 0u; i < 8u; i++)
    {
        out[i] = (((mask >> i) & 0x01u) != 0u) ? '1' : '0';
    }
    out[8] = '\0';
}
