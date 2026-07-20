#include "bsp_adc.h"
#include "adc.h"

#define VREFINT_CAL_ADDR  ((uint16_t *)0x1FFFF7BA)

/* VDDA 缓存，默认 3300mV */
static uint32_t vdda_mv = 3300;

/**
  * @brief  ADC 驱动初始化
  * @note   必须在 MX_ADC1_Init() 之后调用。
  *         1. 设 ADC 时钟为 PCLK2/6 (CubeMX F1 不自动配此分频)
  *         2. 使能 VREFINT 通道 (F1 需手动置 TSVREFE)
  *         3. 硬件自校准
  *         4. 读 VREFINT 反算 VDDA
  */
void BSP_ADC_Init(void)
{
    MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);
    /* ADC_CLK = 64MHz / 6 ≈ 10.67MHz (< 14MHz max) */

    SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE);
    HAL_Delay(1);

    HAL_ADCEx_Calibration_Start(&hadc1);

    uint8_t  adc_ok;
    uint16_t vrefint_raw = BSP_ADC_ReadRaw(ADC_CHANNEL_VREFINT, &adc_ok);
    if (adc_ok && vrefint_raw > 0) {
        vdda_mv = 3300UL * (*VREFINT_CAL_ADDR) / vrefint_raw;
    }
}

/**
  * @brief  单通道轮询读取 ADC 原始值
  * @param  channel: ADC 通道号
  * @param  ok:      [out] 1=成功, 0=失败, 可传 NULL
  * @retval 12-bit 原始值 (0~4095)，失败返回 0
  */
uint16_t BSP_ADC_ReadRaw(uint32_t channel, uint8_t *ok)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    if (ok) *ok = 0;

    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return 0;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    if (ok) *ok = 1;
    return raw;
}

/**
  * @brief  单通道读取电压 (mV)
  */
uint32_t BSP_ADC_ReadmV(uint32_t channel)
{
    uint8_t ok;
    uint16_t raw = BSP_ADC_ReadRaw(channel, &ok);
    if (!ok) return 0;
    return (uint32_t)raw * vdda_mv / 4095UL;
}

/**
  * @brief  获取 VDDA (mV)
  */
uint32_t BSP_ADC_GetVDDA(void)
{
    return vdda_mv;
}

/**
  * @brief  多次采样取平均，返回 ADC_Value_t
  * @note   source 和 channel 字段需调用者手动填入
  */
ADC_Value_t BSP_ADC_ReadChannel(uint32_t channel, uint8_t samples)
{
    ADC_Value_t v = {0};
    v.valid = 0;

    if (samples < ADC_AVG_SAMPLES_MIN || samples > ADC_AVG_SAMPLES_MAX)
        return v;

    uint32_t sum = 0;
    uint8_t  ok;

    for (uint8_t i = 0; i < samples; i++) {
        uint16_t raw = BSP_ADC_ReadRaw(channel, &ok);
        if (!ok) return v;
        sum += raw;
    }

    v.raw   = (uint16_t)((sum + samples / 2) / samples);
    v.mv    = (uint32_t)v.raw * vdda_mv / 4095UL;
    v.valid = 1;
    return v;
}

/**
  * @brief  比较两个 ADC_Value_t，|mV差值| ≤ tol_mV 返回 1
  */
uint8_t BSP_ADC_Compare(ADC_Value_t *a, ADC_Value_t *b, uint32_t tol_mV)
{
    if (!a || !b)                    return 0;
    if (!a->valid || !b->valid)      return 0;

    uint32_t diff = (a->mv > b->mv) ? (a->mv - b->mv) : (b->mv - a->mv);
    return (diff <= tol_mV) ? 1 : 0;
}
