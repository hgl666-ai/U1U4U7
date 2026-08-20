#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"

/* OUT0/1/4/5 (PA0/PA1/PB0/PB1) 已改为 ADC 输入 — 见 bsp_adc.h */

/*===== U7 子板电源使能 =====*/
#define U7_EN_Pin   GPIO_PIN_3
#define U7_EN_Port  GPIOB

/*===== 系统 LED (PC13) =====*/
#define SYS_LED_Pin   GPIO_PIN_13
#define SYS_LED_Port  GPIOC

/*===== RGB LED =====*/
#define LED_R_Pin   GPIO_PIN_12
#define LED_R_Port  GPIOB
#define LED_G_Pin   GPIO_PIN_13
#define LED_G_Port  GPIOB
#define LED_B_Pin   GPIO_PIN_14
#define LED_B_Port  GPIOB

/*===== 预留输出 =====*/
#define RESV_PB4_Pin   GPIO_PIN_4
#define RESV_PB4_Port  GPIOB
#define RESV_PB5_Pin   GPIO_PIN_5
#define RESV_PB5_Port  GPIOB

/*===== 测试 =====*/
uint8_t  BSP_GPIO_Test(void);         /* 自测: RGB+系统LED依次亮 */

/*===== 原子操作 =====*/
void     BSP_GPIO_Init(void);
void     BSP_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t level);
uint8_t  BSP_GPIO_Read(GPIO_TypeDef *port, uint16_t pin);
void     BSP_GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin);

/*===== 语义化封装 =====*/
void BSP_U7_Enable(void);
void BSP_U7_Disable(void);
void BSP_SysLED_On(void);
void BSP_SysLED_Off(void);
void BSP_SysLED_Toggle(void);
void BSP_RGB_Set(uint8_t r, uint8_t g, uint8_t b);

#endif
