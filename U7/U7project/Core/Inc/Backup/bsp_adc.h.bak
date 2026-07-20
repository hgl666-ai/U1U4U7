#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "main.h"

/*===== ADC 通道映射 =====*/
#define ADC_CH_ADC0   ADC_CHANNEL_8   /* PB0 <- U3 运放 <- ADC0 */
#define ADC_CH_ADC1   ADC_CHANNEL_9   /* PB1 <- U3 运放 <- ADC1 */
#define ADC_CH_ADC4   ADC_CHANNEL_0   /* PA0 <- U6 运放 <- ADC4 */
#define ADC_CH_ADC5   ADC_CHANNEL_1   /* PA1 <- U6 运放 <- ADC5 */

/* 采样次数 */
#define ADC_AVG_SAMPLES_MIN   1
#define ADC_AVG_SAMPLES_MAX  32

/*===== 统一结构体 =====*/
typedef enum {
    ADC_SRC_U7 = 0,
    ADC_SRC_U1 = 1,
    ADC_SRC_U4 = 2,
} ADC_Source_t;

typedef struct {
    ADC_Source_t source;   /* 数据来源板 */
    uint8_t      channel;  /* 通道号     */
    uint16_t     raw;      /* 原始值     */
    uint32_t     mv;       /* 电压 mV    */
    uint8_t      valid;    /* 1=成功     */
} ADC_Value_t;

/*===== API =====*/
void         BSP_ADC_Init(void);
uint16_t     BSP_ADC_ReadRaw(uint32_t channel, uint8_t *ok);
uint32_t     BSP_ADC_ReadmV(uint32_t channel);
uint32_t     BSP_ADC_GetVDDA(void);
ADC_Value_t  BSP_ADC_ReadChannel(uint32_t channel, uint8_t samples);
uint8_t      BSP_ADC_Compare(ADC_Value_t *a, ADC_Value_t *b, uint32_t tol_mV);

#endif
