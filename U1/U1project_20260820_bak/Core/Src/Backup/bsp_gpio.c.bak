#include "bsp_gpio.h"

/**
  * @brief  GPIO 应用层统一初始化
  * @note   将所有应用层管理的输出引脚设为默认安全电平。
  *         引脚模式已在 CubeMX 生成的 MX_GPIO_Init() 中配置，
  *         此处仅负责设置初始输出状态，不更改引脚模式。
  */
void BSP_GPIO_Init(void)
{
    /* U7 子板默认断电 */
    BSP_U7_Disable();

    /* 系统 LED 默认熄灭 */
    BSP_SysLED_Off();

    /* RGB LED 全灭 */
    BSP_RGB_Set(0, 0, 0);

    /* OUT0/1/4/5 已改为 ADC 输入，GPIO 不操作 */

    /* 预留引脚拉低 */
    HAL_GPIO_WritePin(RESV_PB4_Port, RESV_PB4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RESV_PB5_Port, RESV_PB5_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  写 GPIO 输出电平
  * @param  port:  GPIO 端口号 (GPIOA / GPIOB / GPIOC)
  * @param  pin:   GPIO 引脚号 (GPIO_PIN_x)
  * @param  level: 0 = 低电平, 非 0 = 高电平
  */
void BSP_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t level)
{
    HAL_GPIO_WritePin(port, pin, (GPIO_PinState)(level ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

/**
  * @brief  读 GPIO 输入电平
  * @param  port: GPIO 端口号
  * @param  pin:  GPIO 引脚号
  * @retval 0 = 低电平, 1 = 高电平
  */
uint8_t BSP_GPIO_Read(GPIO_TypeDef *port, uint16_t pin)
{
    return (uint8_t)HAL_GPIO_ReadPin(port, pin);
}

/**
  * @brief  翻转 GPIO 输出电平
  * @param  port: GPIO 端口号
  * @param  pin:  GPIO 引脚号
  */
void BSP_GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_TogglePin(port, pin);
}



/**
  * @brief  使能 U7 子板电源
  */
void BSP_U7_Enable(void)
{
    HAL_GPIO_WritePin(U7_EN_Port, U7_EN_Pin, GPIO_PIN_SET);
}

/**
  * @brief  关闭 U7 子板电源
  */
void BSP_U7_Disable(void)
{
    HAL_GPIO_WritePin(U7_EN_Port, U7_EN_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  系统心跳 LED 亮
  */
void BSP_SysLED_On(void)
{
    HAL_GPIO_WritePin(SYS_LED_Port, SYS_LED_Pin, GPIO_PIN_SET);
}

/**
  * @brief  系统心跳 LED 灭
  */
void BSP_SysLED_Off(void)
{
    HAL_GPIO_WritePin(SYS_LED_Port, SYS_LED_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  系统心跳 LED 翻转
  */
void BSP_SysLED_Toggle(void)
{
    HAL_GPIO_TogglePin(SYS_LED_Port, SYS_LED_Pin);
}

/**
  * @brief  同步设置 RGB LED 三路电平
  * @param  r: 红色通道, 0 = 灭, 非 0 = 亮
  * @param  g: 绿色通道, 0 = 灭, 非 0 = 亮
  * @param  b: 蓝色通道, 0 = 灭, 非 0 = 亮
  * @note   仅提供单次电平设定；PWM 呼吸灯等效果由任务8上层实现
  */
void BSP_RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    HAL_GPIO_WritePin(LED_R_Port, LED_R_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_Port, LED_G_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_Port, LED_B_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* BSP_OUT_Set 已删除 — OUT0/1/4/5 全部改为 ADC 输入，不再有 GPIO 输出功能 */

/**
  * @brief  自测: 依次点亮 RGB 三色和系统 LED, 各亮 500ms
  * @retval 1=通过
  * @note   目视确认各 LED 依次闪烁即可
  */
uint8_t BSP_GPIO_Test(void)
{
    /* 红灯亮 500ms */
    BSP_RGB_Set(1, 0, 0);
    HAL_Delay(500);

    /* 绿灯亮 500ms */
    BSP_RGB_Set(0, 1, 0);
    HAL_Delay(500);

    /* 蓝灯亮 500ms */
    BSP_RGB_Set(0, 0, 1);
    HAL_Delay(500);

    /* 全灭, 系统 LED 亮 500ms */
    BSP_RGB_Set(0, 0, 0);
    BSP_SysLED_On();
    HAL_Delay(500);
    BSP_SysLED_Off();

    return 1;   /* GPIO 无硬件反馈, 目视确认 */
}
