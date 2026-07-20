#include "bsp_gpio.h"

/**
  * @brief  GPIO 应用层初始化
  * @note   在 MX_GPIO_Init() 之后调用，仅设置安全初始电平。
  *         引脚模式已在 CubeMX 中配好，此处不改。
  */
void BSP_GPIO_Init(void)
{
    /* OUT2/3/6/7 全部改为 ADC 输入，GPIO 不操作 */

    /* 电机默认脱机，方向 CW */
    BSP_EN2_Set(1);
    BSP_DIR2_Set(0);

    /* 系统 LED 默认灭 */
    BSP_SysLED_Off();
}



void BSP_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t level)
{
    HAL_GPIO_WritePin(port, pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BSP_GPIO_Read(GPIO_TypeDef *port, uint16_t pin)
{
    return (uint8_t)HAL_GPIO_ReadPin(port, pin);
}

void BSP_GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_TogglePin(port, pin);
}



/**
  * @brief  电机使能控制
  * @param  en: 0=使能(ENN低), 1=脱机(ENN高)
  */
void BSP_EN2_Set(uint8_t en)
{
    HAL_GPIO_WritePin(EN2_Port, EN2_Pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  电机方向控制
  * @param  dir: 0=CW, 1=CCW
  */
void BSP_DIR2_Set(uint8_t dir)
{
    HAL_GPIO_WritePin(DIR2_Port, DIR2_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}



/**
  * @brief  读取 INx 输入电平
  * @param  idx: 输入编号 (1/2)
  * @retval 0=低, 1=高, 非法 idx 返回 0
  */
uint8_t BSP_IN_Read(uint8_t idx)
{
    switch (idx) {
        case 1:  return (uint8_t)HAL_GPIO_ReadPin(IN1_Port,  IN1_Pin);
        case 2:  return (uint8_t)HAL_GPIO_ReadPin(IN2_Port,  IN2_Pin);
        default: return 0;
    }
}



void BSP_SysLED_On(void)
{
    HAL_GPIO_WritePin(SYS_LED_Port, SYS_LED_Pin, GPIO_PIN_SET);
}

void BSP_SysLED_Off(void)
{
    HAL_GPIO_WritePin(SYS_LED_Port, SYS_LED_Pin, GPIO_PIN_RESET);
}

void BSP_SysLED_Toggle(void)
{
    HAL_GPIO_TogglePin(SYS_LED_Port, SYS_LED_Pin);
}
