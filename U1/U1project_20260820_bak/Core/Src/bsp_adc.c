#include "bsp_adc.h"
#include "adc.h"

#define VREFINT_CAL_ADDR    ((uint16_t *)0x1FFFF7BA)

/* 缓存校准后的 VDDA 供电电压 (mV)，默认 3300mV */
static uint32_t vdda_mv = 3300;

/**
  * @brief  ADC 驱动初始化
  * @note   必须在 CubeMX 生成的 MX_ADC1_Init() 之后调用。
  *         1. 使能内部 VREFINT 通道 (STM32F1 需手动置 TSVREFE)
  *         2. 执行 ADC 硬件自校准
  *         3. 读取 VREFINT 反算实际 VDDA 电压并缓存
  */
void BSP_ADC_Init(void)
{
    SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE);

    
    HAL_Delay(1);

    HAL_ADCEx_Calibration_Start(&hadc1);

    /* 读取 VREFINT 原始值，反算实际 VDDA 电压 */
    uint8_t adc_ok;
    uint16_t vrefint_raw = BSP_ADC_ReadRaw(ADC_CHANNEL_VREFINT, &adc_ok);
    if (adc_ok && vrefint_raw > 0) {
        uint16_t vrefint_cal = *VREFINT_CAL_ADDR;

        vdda_mv = 3300UL * vrefint_cal / vrefint_raw;
    }
    /* 若 ADC 读取失败，保留默认值 3300mV */
}

/**
  * @brief  自测: 读 VREFINT 校准 VDDA, 期望 3000~3600mV
  * @retval 1=通过, 0=失败
  */
uint8_t BSP_ADC_Test(void)
{
    BSP_ADC_Init();
    uint32_t v = BSP_ADC_GetVDDA();
    return (v >= 3000 && v <= 3600) ? 1 : 0;
}

/**
  * @brief  单通道轮询读取 ADC 原始值
  * @param  channel: ADC 通道号
  * @retval 12-bit ADC 原始值 (0 ~ 4095)
  * @note   采样时间 239.5 周期，适合中低速信号。
  *         每次调用会覆盖 hadc1 的 Rank 1 配置。
  */
uint16_t BSP_ADC_ReadRaw(uint32_t channel, uint8_t *ok)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    if (ok) *ok = 0;  /* 默认失败 */

    /* 动态配置通道*/
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0;
    }

    HAL_ADC_Start(&hadc1);

    /* 10ms 超时：ADC 时钟 12MHz，239.5 周期 ≈ 20μs/次，留足余量 */
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    uint16_t raw = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    if (ok) *ok = 1;  /* 成功 */
    return raw;
}

/**
  * @brief  单通道读取电压值
  * @param  channel: ADC 通道号
  * @retval 电压值 (mV)，基于 BSP_ADC_Init 中校准的 VDDA 计算
  */
uint32_t BSP_ADC_ReadmV(uint32_t channel)
{
    uint8_t adc_ok;
    uint16_t raw = BSP_ADC_ReadRaw(channel, &adc_ok);
    if (!adc_ok) return 0;
    return (uint32_t)raw * vdda_mv / 4095UL;
}

/**
  * @brief  获取校准后的 VDDA 供电电压
  * @retval VDDA 电压 (mV)
  */
uint32_t BSP_ADC_GetVDDA(void)
{
    return vdda_mv;
}

/**
  * @brief  多次采样取平均，返回填充完整的 ADC_Value_t
  * @param  channel: ADC 通道号
  * @param  samples: 采样次数 (1~32)
  * @retval ADC_Value_t，含 raw/mV/valid；失败时 valid=0
  * @note   到 APP 层用 source 和 channel 手动填入，此处不设。
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
        if (!ok) return v;                 /* 任一次失败直接返回 valid=0 */
        sum += raw;
    }

    v.raw   = (uint16_t)((sum + samples / 2) / samples);  /* 四舍五入 */
    v.mv    = (uint32_t)v.raw * vdda_mv / 4095UL;
    v.valid = 1;
    return v;
}

/**
  * @brief  比较两个 ADC_Value_t，差值在容差内返回 1
  * @param  a / b: 待比较的两个结构体指针
  * @param  tol_mV: 容许偏差 (mV)
  * @retval 1 = 一致, 0 = 不一致或任一无数据
  */
uint8_t BSP_ADC_Compare(ADC_Value_t *a, ADC_Value_t *b, uint32_t tol_mV)
{
    if (!a || !b)           return 0;
    if (!a->valid || !b->valid) return 0;

    uint32_t diff = (a->mv > b->mv) ? (a->mv - b->mv) : (b->mv - a->mv);
    return (diff <= tol_mV) ? 1 : 0;
}

/**
  * @brief  将 GPIO 引脚切换为模拟输入模式
  * @param  port: GPIO 端口号 (GPIOA / GPIOB / GPIOC)
  * @param  pin:  GPIO 引脚号
  * @note   用于 ADC 采集前，将原本作为数字输出的引脚临时切为模拟输入。
  *         调用后引脚输出驱动器关闭，可安全接入外部模拟信号。
  *         典型用法：治具选通 U4 测试点后，将对应 U1 引脚切为模拟模式。
  */
void BSP_ADC_PinToAnalog(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/**
  * @brief  将 GPIO 引脚恢复为推挽输出模式
  * @param  port: GPIO 端口号
  * @param  pin:  GPIO 引脚号
  * @note   用于 ADC 采集完成后，将引脚恢复为 GPIO 输出功能。
  *         恢复后初始电平为低，避免与外部电路产生驱动冲突。
  */
void BSP_ADC_PinToOutput(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
