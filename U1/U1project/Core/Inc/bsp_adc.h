#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "main.h"


/* 测试点 ADC 通道 (均经 LM358 电压跟随) */
#define ADC_CH_ADC0   ADC_CHANNEL_8   /* PB0 <- U2 运放 <- ADC0 */
#define ADC_CH_ADC1   ADC_CHANNEL_9   /* PB1 <- U2 运放 <- ADC1 */
#define ADC_CH_ADC2   ADC_CHANNEL_0   /* PA0 <- U5 运放 <- ADC2 */
#define ADC_CH_ADC3   ADC_CHANNEL_1   /* PA1 <- U5 运放 <- ADC3 */

/* 采样次数 */
#define ADC_AVG_SAMPLES_MIN    1
#define ADC_AVG_SAMPLES_MAX   32

typedef enum {
    ADC_SRC_U1 = 0,
    ADC_SRC_U4 = 1,
    ADC_SRC_U7 = 2,
} ADC_Source_t;

typedef struct {
    ADC_Source_t source;    /* 数据来源板         */
    uint8_t      channel;   /* 通道号             */
    uint16_t     raw;       /* 12-bit 原始值      */
    uint32_t     mv;        /* 电压值 (mV)        */
    uint8_t      valid;     /* 1=读取成功         */
} ADC_Value_t;

/*===== API =====*/
uint8_t  BSP_ADC_Test(void);           /* 自测: 读 VREFINT, 1=通过 */
uint16_t BSP_ADC_ReadRaw(uint32_t channel, uint8_t *ok);
uint32_t BSP_ADC_ReadmV(uint32_t channel);
uint32_t BSP_ADC_GetVDDA(void);

void  BSP_ADC_PinToAnalog(GPIO_TypeDef *port, uint16_t pin);
void  BSP_ADC_PinToOutput(GPIO_TypeDef *port, uint16_t pin);


/*封装*/
void  BSP_ADC_Init(void);
ADC_Value_t BSP_ADC_ReadChannel(uint32_t channel, uint8_t samples);
uint8_t  BSP_ADC_Compare(ADC_Value_t *a, ADC_Value_t *b, uint32_t tol_mV);

#endif
