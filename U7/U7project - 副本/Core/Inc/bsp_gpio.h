#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"



/*===== 电机控制 =====*/
#define EN2_Pin     GPIO_PIN_5
#define EN2_Port    GPIOA
#define DIR2_Pin    GPIO_PIN_4
#define DIR2_Port   GPIOB

/*===== TMC2209 单线 UART =====*/
#define MOTO_TX_Pin   GPIO_PIN_7
#define MOTO_TX_Port  GPIOA
#define MOTO_RX_Pin   GPIO_PIN_6
#define MOTO_RX_Port  GPIOA

/*===== 数字输入 =====*/
#define IN1_Pin     GPIO_PIN_9
#define IN1_Port    GPIOB
#define IN2_Pin     GPIO_PIN_10
#define IN2_Port    GPIOB
#define U7_EN_Pin   GPIO_PIN_11
#define U7_EN_Port  GPIOB


#define SYS_LED_Pin   GPIO_PIN_13
#define SYS_LED_Port  GPIOC


void    BSP_GPIO_Init(void);
void    BSP_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t level);
uint8_t BSP_GPIO_Read(GPIO_TypeDef *port, uint16_t pin);
void    BSP_GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin);

/*===== 语义化封装 =====*/
void BSP_EN2_Set(uint8_t en);          /* 0=使能电机, 1=脱机 */
void BSP_DIR2_Set(uint8_t dir);        /* 0=CW, 1=CCW */
uint8_t BSP_IN_Read(uint8_t idx);
void BSP_SysLED_On(void);
void BSP_SysLED_Off(void);
void BSP_SysLED_Toggle(void);

#endif
